package frc.robot.systems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.GeomUtil;
import frc.lib.telemetry.Telemetry;
import frc.robot.Constants;
import frc.robot.game.FieldConstants;
import frc.robot.systems.shooter.ShooterConstants.FlywheelSpeedSample;
import frc.robot.systems.shooter.ShooterConstants.HoodAngleSample;
import frc.robot.systems.shooter.ShooterConstants.TimeOfFlightSample;

public class ShotCalculator {
  private static ShotCalculator mInstance;

  // Smoothing for setpoint rates
  private final LinearFilter mHeadingRateFilter = LinearFilter.movingAverage((int) (0.1 / Constants.kPeriodSecs));
  private final LinearFilter mHoodRateFilter = LinearFilter.movingAverage((int) (0.1 / Constants.kPeriodSecs));

  private Rotation2d mLastDesiredHeading;
  private double mLastHoodAngleRad = Double.NaN;

  private Rotation2d mDesiredHeadingField;
  private Rotation2d mHeadingError; // desired - current (wrapped)
  private double mDesiredHeadingRateRadPerSec;
  private double mHoodRateRadPerSec;

  public static ShotCalculator getInstance() {
    if (mInstance == null) mInstance = new ShotCalculator();
    return mInstance;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d desiredRobotHeadingField,
      Rotation2d headingError,
      double desiredHeadingRateRadPerSec,
      double hoodAngleRad,
      double hoodRateRadPerSec,
      double flywheelSpeed) {}

  private ShootingParameters mLatestParameters = null;
  private static final double mPhaseDelay = 0.03;

  private static final InterpolatingTreeMap<Double, Rotation2d> mShotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap mShotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap mTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    for (HoodAngleSample sample : ShooterConstants.kHoodAngleSamples)
        mShotHoodAngleMap.put(sample.distanceMeters(),Rotation2d.fromDegrees(sample.hoodAngleDeg()));

    for (FlywheelSpeedSample sample : ShooterConstants.kFlywheelSpeedSamples)
        mShotFlywheelSpeedMap.put(sample.distanceMeters(), sample.flywheelSpeed());

    for (TimeOfFlightSample sample : ShooterConstants.kTimeOfFlightSamples)
        mTimeOfFlightMap.put(sample.distanceMeters(),sample.timeSeconds());
  }

  /**
   * shooterYawOffset is the fixed yaw of shooter relative to robot forward.
   * Example: shooter points forward -> Rotation2d.kZero
   * Example: shooter points left -> Rotation2d.fromDegrees(90)
   */
  public ShootingParameters getParameters(
    Pose2d pEstimatedPose,
    ChassisSpeeds pRobotVelocityRobotRelative,
    ChassisSpeeds pFieldVelocityFieldRelative,
    Rotation2d pShooterYawOffset) {
        if (mLatestParameters != null) return mLatestParameters;

        Pose2d estimatedPose = predictPoseAfterDelay(pEstimatedPose, pRobotVelocityRobotRelative);

        Translation2d target = getHubTargetTranslation();

        Pose2d shooterPose = getShooterPoseField(estimatedPose);

        ShooterFieldVelocity shooterVel = getShooterFieldVelocity(estimatedPose, pFieldVelocityFieldRelative);

        LookaheadResult lookahead = solveLookahead(shooterPose, target, shooterVel);

        Rotation2d shooterAimField = computeAimFieldAngle(target, lookahead.lookaheadPose());
        mDesiredHeadingField = computeDesiredRobotHeadingField(shooterAimField, pShooterYawOffset);

        mHeadingError = computeHeadingError(mDesiredHeadingField, estimatedPose.getRotation());

        ShotSetpoints shot = getShotSetpoints(lookahead.lookaheadDistance());

        updateRates(mDesiredHeadingField, shot.hoodAngleRad());

        boolean valid = isDistanceInValidRange(lookahead.lookaheadDistance());

        mLatestParameters =
            new ShootingParameters(
                valid,
                mDesiredHeadingField,
                mHeadingError,
                mDesiredHeadingRateRadPerSec,
                shot.hoodAngleRad(),
                mHoodRateRadPerSec,
                shot.flywheelSpeed());

        Telemetry.log("ShotCalculator/LookaheadPose", lookahead.lookaheadPose());
        Telemetry.log("ShotCalculator/LookaheadDistance", lookahead.lookaheadDistance());
        Telemetry.log("ShotCalculator/DesiredHeadingField", mDesiredHeadingField);
        Telemetry.log("ShotCalculator/HeadingError", mHeadingError);

        return mLatestParameters;
    }

    private record ShooterFieldVelocity(double vx, double vy) {}

    private record LookaheadResult(Pose2d lookaheadPose, double lookaheadDistance) {}

    private record ShotSetpoints(double hoodAngleRad, double flywheelSpeed) {}

    /** Predicts robot pose after phase delay using robot-relative chassis speeds. */
    private Pose2d predictPoseAfterDelay(Pose2d pose, ChassisSpeeds robotRelativeVel) {
        return pose.exp(
            new Twist2d(
                robotRelativeVel.vxMetersPerSecond * mPhaseDelay,
                robotRelativeVel.vyMetersPerSecond * mPhaseDelay,
                robotRelativeVel.omegaRadiansPerSecond * mPhaseDelay));
    }

    /** Returns the hub target location in field coordinates (alliance-correct). */
    private Translation2d getHubTargetTranslation() {
        return AllianceFlipUtil.apply(FieldConstants.kHubOuterPose.getTranslation().toTranslation2d());
    }

    /** Returns shooter origin pose in field coordinates (robot pose + fixed shooter offset). */
    private Pose2d getShooterPoseField(Pose2d robotPoseField) {
        return robotPoseField.transformBy(
            GeomUtil.toTransform2d(Constants.RobotConstants.kMiddleShooterOffset));
    }

    /**
     * Computes shooter linear velocity in the field frame, including omega×r contribution
     * from the shooter’s offset relative to robot center.
     */
    private ShooterFieldVelocity getShooterFieldVelocity(Pose2d robotPoseField, ChassisSpeeds fieldVel) {
        double robotAngle = robotPoseField.getRotation().getRadians();

        double offX = Constants.RobotConstants.kMiddleShooterOffset.getX();
        double offY = Constants.RobotConstants.kMiddleShooterOffset.getY();

        double shooterVelX =
            fieldVel.vxMetersPerSecond
                + fieldVel.omegaRadiansPerSecond * (offY * Math.cos(robotAngle) - offX * Math.sin(robotAngle));

        double shooterVelY =
            fieldVel.vyMetersPerSecond
                + fieldVel.omegaRadiansPerSecond * (offX * Math.cos(robotAngle) - offY * Math.sin(robotAngle));

        return new ShooterFieldVelocity(shooterVelX, shooterVelY);
    }

    /**
     * Iteratively solves the “lookahead” shooter translation used for aiming while moving.
     * Uses time-of-flight(distance) to offset shooter position by shooterVelocity * TOF.
     */
    private LookaheadResult solveLookahead(Pose2d pShooterPose, Translation2d pTarget, ShooterFieldVelocity pShooterVel) {
        Pose2d lookaheadPose = pShooterPose;
        double lookaheadDistance = pTarget.getDistance(pShooterPose.getTranslation());

        for (int i = 0; i < 20; i++) {
            double dForLookup = MathUtil.clamp(lookaheadDistance, ShooterConstants.kMinTofDistanceMeters, ShooterConstants.kMaxTofDistanceMeters);

            double tof = mTimeOfFlightMap.get(dForLookup);

            Translation2d lookaheadTranslation = pShooterPose.getTranslation().plus(new Translation2d(pShooterVel.vx() * tof, pShooterVel.vy() * tof));

            lookaheadPose = new Pose2d(lookaheadTranslation, pShooterPose.getRotation());
            lookaheadDistance = pTarget.getDistance(lookaheadPose.getTranslation());
        }

        return new LookaheadResult(lookaheadPose, lookaheadDistance);
    }

    /** Field angle from lookahead shooter translation to the target. */
    private Rotation2d computeAimFieldAngle(Translation2d pTarget, Pose2d pLookaheadPose) {
        return pTarget.minus(pLookaheadPose.getTranslation()).getAngle();
    }

    /** Converts required shooter aim angle into required robot heading (fixed shooter). */
    private Rotation2d computeDesiredRobotHeadingField(Rotation2d pShooterAimField, Rotation2d pShooterYawOffset) {
        return pShooterAimField.minus(pShooterYawOffset);
    }

    /** Computes wrapped heading error (desired - current) in [-pi, pi]. */
    private Rotation2d computeHeadingError(Rotation2d pDesiredHeading, Rotation2d pCurrentHeading) {
        double errorRad = MathUtil.angleModulus(pDesiredHeading.minus(pCurrentHeading).getRadians());
        return Rotation2d.fromRadians(errorRad);
    }

    /** Returns hood angle + flywheel setpoint from the tuned interpolation maps. */
    private ShotSetpoints getShotSetpoints(double pLookaheadDistance) {
        double dShot =
            MathUtil.clamp(
                pLookaheadDistance,
                ShooterConstants.kMinValidShotDistanceMeters,
                ShooterConstants.kMaxValidShotDistanceMeters);

        double hoodAngleRad = mShotHoodAngleMap.get(dShot).getRadians();
        double flywheelSpeed = mShotFlywheelSpeedMap.get(dShot);

        return new ShotSetpoints(hoodAngleRad, flywheelSpeed);
    }

    /** Updates filtered rates for heading and hood. */
    private void updateRates(Rotation2d pDesiredHeadingField, double pHoodAngleRad) {
        if (mLastDesiredHeading == null) mLastDesiredHeading = pDesiredHeadingField;
        if (Double.isNaN(mLastHoodAngleRad)) mLastHoodAngleRad = pHoodAngleRad;

        mDesiredHeadingRateRadPerSec =
            mHeadingRateFilter.calculate(
                MathUtil.angleModulus(pDesiredHeadingField.minus(mLastDesiredHeading).getRadians())
                    / Constants.kPeriodSecs);

        mHoodRateRadPerSec =
            mHoodRateFilter.calculate((pHoodAngleRad - mLastHoodAngleRad) / Constants.kPeriodSecs);

        mLastDesiredHeading = pDesiredHeadingField;
        mLastHoodAngleRad = pHoodAngleRad;
    }

    private boolean isDistanceInValidRange(double pDistanceMeters) {
    return pDistanceMeters >= ShooterConstants.kMinValidShotDistanceMeters
        && pDistanceMeters <= ShooterConstants.kMaxValidShotDistanceMeters;
    }

    public void clearShootingParameters() {
        mLatestParameters = null;
    }
}
