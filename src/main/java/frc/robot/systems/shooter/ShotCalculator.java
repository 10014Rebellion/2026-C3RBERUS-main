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

  private double mHoodAngleRad = Double.NaN;
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

    // Predict pose after phase delay
    Pose2d estimatedPose =
        pEstimatedPose.exp(
            new Twist2d(
                pRobotVelocityRobotRelative.vxMetersPerSecond * mPhaseDelay,
                pRobotVelocityRobotRelative.vyMetersPerSecond * mPhaseDelay,
                pRobotVelocityRobotRelative.omegaRadiansPerSecond * mPhaseDelay));

    // Target (field)
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.kHubOuterPose.getTranslation().toTranslation2d());

    // Shooter position (field): robot pose + fixed offset to shooter exit (or shooter pivot)
    Pose2d shooterPose =
        estimatedPose.transformBy(
            GeomUtil.toTransform2d(Constants.RobotConstants.kMiddleShooterOffset));

    double shooterToTargetDistance = target.getDistance(shooterPose.getTranslation());

    // Compute shooter linear velocity in the field frame (robot translation + omega cross offset)
    ChassisSpeeds robotFieldVel = pFieldVelocityFieldRelative;
    double robotAngle = estimatedPose.getRotation().getRadians();

    double offX = Constants.RobotConstants.kMiddleShooterOffset.getX();
    double offY = Constants.RobotConstants.kMiddleShooterOffset.getY();

    double shooterVelX =
        robotFieldVel.vxMetersPerSecond
            + robotFieldVel.omegaRadiansPerSecond * (offY * Math.cos(robotAngle) - offX * Math.sin(robotAngle));
    double shooterVelY =
        robotFieldVel.vyMetersPerSecond
            + robotFieldVel.omegaRadiansPerSecond * (offX * Math.cos(robotAngle) - offY * Math.sin(robotAngle));

    // Lookahead: iterate because time-of-flight depends on distance
    Pose2d lookaheadPose = shooterPose;
    double lookaheadDistance = shooterToTargetDistance;

    for (int i = 0; i < 20; i++) {
      double dForLookup = MathUtil.clamp(lookaheadDistance, ShooterConstants.kMinValidShotDistanceMeters, ShooterConstants.kMaxValidShotDistanceMeters);
      double tof = mTimeOfFlightMap.get(dForLookup);

      Translation2d lookaheadTranslation =
          shooterPose.getTranslation().plus(new Translation2d(shooterVelX * tof, shooterVelY * tof));

      lookaheadPose = new Pose2d(lookaheadTranslation, shooterPose.getRotation());
      lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // This is the field angle the shooter must point at
    Rotation2d shooterAimField = target.minus(lookaheadPose.getTranslation()).getAngle();

    // Convert to robot heading setpoint (because shooter is fixed on robot)
    mDesiredHeadingField = shooterAimField.minus(pShooterYawOffset);

    // Heading error (wrapped to [-pi, pi]) for convenience
    double errorRad =
        MathUtil.angleModulus(mDesiredHeadingField.minus(estimatedPose.getRotation()).getRadians());
    mHeadingError = Rotation2d.fromRadians(errorRad);

    // Hood and flywheel from distance
    double dShot = MathUtil.clamp(lookaheadDistance, ShooterConstants.kMinValidShotDistanceMeters, ShooterConstants.kMaxValidShotDistanceMeters);
    mHoodAngleRad = mShotHoodAngleMap.get(dShot).getRadians();
    double flywheelSpeed = mShotFlywheelSpeedMap.get(dShot);

    // Rates
    if (mLastDesiredHeading == null) mLastDesiredHeading = mDesiredHeadingField;
    if (Double.isNaN(mLastHoodAngleRad)) mLastHoodAngleRad = mHoodAngleRad;

    mDesiredHeadingRateRadPerSec =
        mHeadingRateFilter.calculate(
            MathUtil.angleModulus(mDesiredHeadingField.minus(mLastDesiredHeading).getRadians())
                / Constants.kPeriodSecs);

    mHoodRateRadPerSec =
        mHoodRateFilter.calculate((mHoodAngleRad - mLastHoodAngleRad) / Constants.kPeriodSecs);

    mLastDesiredHeading = mDesiredHeadingField;
    mLastHoodAngleRad = mHoodAngleRad;

    boolean valid = lookaheadDistance >= ShooterConstants.kMinValidShotDistanceMeters && lookaheadDistance <= ShooterConstants.kMaxValidShotDistanceMeters;

    mLatestParameters =
        new ShootingParameters(
            valid,
            mDesiredHeadingField,
            mHeadingError,
            mDesiredHeadingRateRadPerSec,
            mHoodAngleRad,
            mHoodRateRadPerSec,
            flywheelSpeed);

    Telemetry.log("ShotCalculator/LookaheadPose", lookaheadPose);
    Telemetry.log("ShotCalculator/LookaheadDistance", lookaheadDistance);
    Telemetry.log("ShotCalculator/DesiredHeadingField", mDesiredHeadingField);
    Telemetry.log("ShotCalculator/HeadingError", mHeadingError);

    return mLatestParameters;
  }

  public void clearShootingParameters() {
    mLatestParameters = null;
  }
}
