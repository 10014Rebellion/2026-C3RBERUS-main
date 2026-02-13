// REBELLION 10014

package frc.robot.systems.drive;

import static frc.robot.systems.drive.DriveConstants.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.GeomUtil;
import frc.lib.optimizations.PPRobotConfigLoader;
import frc.lib.pathplanner.AzimuthFeedForward;
import frc.lib.pathplanner.SwerveSetpoint;
import frc.lib.pathplanner.SwerveSetpointGenerator;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.apriltag.ATagVision;
import frc.robot.systems.apriltag.ATagVision.VisionObservation;
import frc.robot.systems.drive.controllers.SpeedErrorController;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroInputsAutoLogged;
import frc.robot.systems.drive.modules.Module;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private final Module[] mModules;
    private final GyroIO mGyro;
    private final GyroInputsAutoLogged mGyroInputs = new GyroInputsAutoLogged();
    private final ATagVision mVision;

    private Rotation2d mRobotRotation;
    private final SwerveDriveOdometry mOdometry;
    private final SwerveDrivePoseEstimator mPoseEstimator;
    private final Field2d mField = new Field2d();
    private final Debouncer mSkidFactorDebouncer = new Debouncer(0.25, DebounceType.kFalling);
    private final Debouncer mCollisionDebouncer = new Debouncer(0.25, DebounceType.kFalling);

    public static RobotConfig mRobotConfig;
    private final SwerveSetpointGenerator mSetpointGenerator;
    private SwerveSetpoint mPreviousSetpoint =
            new SwerveSetpoint(
                new ChassisSpeeds(), 
                SwerveHelper.zeroStates(), 
                DriveFeedforwards.zeros(4), 
                AzimuthFeedForward.zeros());

    private ChassisSpeeds mDesiredSpeeds = new ChassisSpeeds();
    private DriveFeedforwards mPathPlanningFF = DriveFeedforwards.zeros(4);
    private DriveFeedforwards mDefaultFF = DriveFeedforwards.zeros(4);
    @AutoLogOutput(key = "Drive/Feedforward/Choreo")
    private boolean mUseChoreoFeedForward = false;
    @AutoLogOutput(key = "Drive/Feedforward/Filter")
    private boolean mFilterFeedForward = false;
    private final PathConstraints mDriveConstraints = DriveConstants.kAutoConstraints;

    private SwerveModulePosition[] mPrevPositions = SwerveHelper.zeroPositions();
    private Rotation2d[] mAngleDeltas = new Rotation2d[4];
    @AutoLogOutput(key="Drive/Swerve/PreviousDriveAmps")
    private double[] mPrevDriveAmps = new double[] {0.0, 0.0, 0.0, 0.0};

    private final boolean kUseGenerator = true;
    private final SpeedErrorController mSpeedErrorController = new SpeedErrorController();

    private DriveManager mDriveManager;

    public static final LoggedTunableNumber tDriftRate = new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);
    public static final LoggedTunableNumber tDriveFFAggressiveness = new LoggedTunableNumber("Drive/Teleop/DriveFFAggressiveness", kDriveFFAggressiveness);

    public static final LoggedTunableNumber tRotationDriftTestSpeedDeg = new LoggedTunableNumber("Drive/DriftRotationTestDeg", 360);
    public static final LoggedTunableNumber tLinearTestSpeedMPS = new LoggedTunableNumber("Drive/LinearTestMPS", 4.5);
    public static final LoggedTunableNumber tAzimuthCharacterizationVoltage = new LoggedTunableNumber("Drive/AzimuthCharacterizationVoltage", 0);
    public static final LoggedTunableNumber tAzimuthCharacterizationAmps = new LoggedTunableNumber("Drive/AzimuthCharacterizationAmps", 0);
    
    public Drive(Module[] modules, GyroIO gyro, ATagVision vision) {
        this.mModules = modules;
        this.mGyro = gyro;
        this.mVision = vision;

        mRobotRotation = mGyroInputs.iYawPosition;

        mOdometry = new SwerveDriveOdometry(kKinematics, getRobotRotation(), getModulePositions());
        mPoseEstimator = new SwerveDrivePoseEstimator(kKinematics, getRobotRotation(), getModulePositions(), new Pose2d());

        mRobotConfig = PPRobotConfigLoader.load();
        mSetpointGenerator = new SwerveSetpointGenerator(mRobotConfig, kMaxAzimuthAngularRadiansPS);
        mDriveManager = new DriveManager(this);

        PhoenixOdometryThread.getInstance().start();

        AutoBuilder.configure(
            this::getPoseEstimate, 
            this::setPose, 
            this::getRobotChassisSpeeds,
            (speeds, ff) -> {
                mDriveManager.setToAuton();;
                mDriveManager.setPPDesiredSpeeds(speeds);
                mPathPlanningFF = ff;
            },
            new PPHolonomicDriveController(kPPTranslationPID, kPPRotationPID),
            mRobotConfig, 
            () -> AllianceFlipUtil.shouldFlip(), 
            this);

        SwerveHelper.setUpPathPlanner();
        SmartDashboard.putData(mField);
    }

    public DriveManager getDriveManager() {
        return mDriveManager;
    }

    ///// ENTRY POINT TO THE DRIVE \\\\\
    @Override
    public void periodic() {
        updateSensorsAndOdometry();
        runSwerve(mDriveManager.computeDesiredSpeedsFromState());
    }

    private void updateSensorsAndOdometry() {
        try {
            kOdometryLock.lock();
            for (Module module : mModules) module.periodic();

            mGyro.updateInputs(mGyroInputs);
            Logger.processInputs("Drive/Gyro", mGyroInputs);
        } catch (Exception e) {
            Telemetry.reportException(e);
        } finally {
            kOdometryLock.unlock();
        }

        double skidCount = 0;
        double[] sampleTimestamps = mModules[0].getOdometryTimeStamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        mAngleDeltas = SwerveHelper.zeroRotations();
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = SwerveHelper.zeroPositions();
            SwerveModulePosition[] moduleDeltas = SwerveHelper.zeroPositions();

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = mModules[moduleIndex].getOdometryPositions()[i];

                mAngleDeltas[moduleIndex] = mAngleDeltas[moduleIndex].plus(
                    GeomUtil.getSmallestChangeInRotation(
                        modulePositions[moduleIndex].angle, 
                        mPrevPositions[moduleIndex].angle));

                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters 
                        - mPrevPositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);

                mPrevPositions[moduleIndex] = modulePositions[moduleIndex];
            }

            Twist2d robotTwist = kKinematics.toTwist2d(moduleDeltas);
            double skidRatio = SwerveHelper.skidRatio(GeomUtil.toChassisSpeeds(robotTwist, 1.0 / kOdometryFrequency));

            if(kSkidRatioCap < skidRatio) skidCount++;

            // Update gyro angle
            // Use the real gyro angle
            if (mGyroInputs.iConnected) mRobotRotation = mGyroInputs.odometryYawPositions[i];
            // Use the angle delta from the kinematics and module deltas
            else mRobotRotation = mRobotRotation.plus(new Rotation2d(robotTwist.dtheta));

            mPoseEstimator.updateWithTime(sampleTimestamps[i], mRobotRotation, modulePositions);

            Logger.recordOutput("Drive/Odometry/SkidRatio/"+i, skidRatio);
        }

        double skidFactor = ( mSkidFactorDebouncer.calculate(skidCount > 0) ) 
            ? skidCount * kSkidScalar 
            : 0;

        double gyroFactor = ( mCollisionDebouncer.calculate( mGyro.getAccMagG() > kCollisionCapG ) ) 
            ? kCollisionScalar 
            : 1.0;
        
        double visionFactor = skidFactor + gyroFactor;
        
        /* VISION */
        mVision.periodic(mPoseEstimator.getEstimatedPosition(), mOdometry.getPoseMeters());
        VisionObservation[] observations = mVision.getVisionObservations();
        for (VisionObservation observation : observations) {
            if (observation.hasObserved()) {
                mPoseEstimator.addVisionMeasurement(
                    observation.pose(), 
                    observation.timeStamp(), 
                    observation.stdDevs().times(1.0 / visionFactor));
            }

            Telemetry.logVisionObservationStdDevs(observation);
        }

        /* For logging purposes */
        mOdometry.update(mRobotRotation, getModulePositions());
        mField.setRobotPose(getPoseEstimate());
        Logger.recordOutput("Drive/Odometry/SkidFactor", skidFactor);
        Logger.recordOutput("Drive/Odometry/SkidCount", skidCount);
        Logger.recordOutput("Drive/Odometry/GyroFactor", gyroFactor);
        Logger.recordOutput("Drive/Odometry/VisionFactor", visionFactor);
    }

    ////////////// CHASSIS SPEED TO MODULES \\\\\\\\\\\\\\\\
    /* Sets the desired swerve module states to the robot */
    public void runSwerve(ChassisSpeeds speeds) {
        if(speeds == null) return;
        mDesiredSpeeds = mSpeedErrorController.correctSpeed(getRobotChassisSpeeds(), SwerveHelper.discretize(speeds, tDriftRate.get()));

        /* Logs all the possible drive states, great for debugging */
        SwerveHelper.logPossibleDriveStates(
            kDoExtraLogging, 
            mDesiredSpeeds, 
            getModuleStates(), 
            mPreviousSetpoint, 
            mRobotRotation);

        SwerveModuleState[] unOptimizedSetpointStates = new SwerveModuleState[4];
        SwerveModuleState[] setpointStates = kKinematics.toSwerveModuleStates(mDesiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeedMPS);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        mPreviousSetpoint = mSetpointGenerator.generateSetpoint(
            mPreviousSetpoint, 
            mDesiredSpeeds, 
            (DriverStation.isTeleop()) 
                ? mDriveConstraints 
                : kAutoConstraints, 
            SwerveHelper.dt);

        /* Only for logging purposes */
        SwerveModuleState[] moduleTorques = SwerveHelper.zeroStates();

        mDefaultFF = mPreviousSetpoint.feedforwards();
        // Telemetry.log("Drive/Odometry/generatedFieldSpeeds",
        // ChassisSpeeds.fromRobotRelativeSpeeds(previousSetpoint.robotRelativeSpeeds(), robotRotation));
        for (int i = 0; i < 4; i++) {
            if (kUseGenerator) {
                /* Logs the drive feedforward stuff */
                SwerveHelper.logDriveFeedforward(mDefaultFF, i);

                setpointStates[i] = new SwerveModuleState(
                    mPreviousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    /* setpointAngle = currentAngle if the speed is less than 0.01 */
                    SwerveHelper.removeAzimuthJitter(
                        mPreviousSetpoint.moduleStates()[i], 
                        mModules[i].getCurrentState()));
                unOptimizedSetpointStates[i] = SwerveHelper.copyState(setpointStates[i]);

                setpointStates[i].optimize(mModules[i].getCurrentState().angle);

                /* Feedforward cases based on driveState */
                double driveAmps = 
                    calculateDriveFeedforward(unOptimizedSetpointStates, i) + SwerveHelper.deadReckoningFeedforward(mAngleDeltas[i]);
                double desiredAzimuthVelocityRadPS = 
                    mPreviousSetpoint.azimuthFeedforwards().azimuthSpeedRadiansPS()[i];

                // Multiplies by cos(angleError) to stop the drive from going in the wrong direction
                setpointStates[i].cosineScale(mModules[i].getCurrentState().angle);

                optimizedSetpointStates[i] = mModules[i].setDesiredStateWithAmpFF(
                    setpointStates[i], 
                    driveAmps, 
                    desiredAzimuthVelocityRadPS);

                /* Normalized for logging */
                moduleTorques[i] = new SwerveModuleState(
                    (driveAmps * kMaxLinearSpeedMPS) 
                        / kDriveFOCAmpLimit, 
                    optimizedSetpointStates[i].angle);
            } else {
                setpointStates[i] = new SwerveModuleState(
                    setpointStates[i].speedMetersPerSecond,
                    SwerveHelper.removeAzimuthJitter(
                        setpointStates[i], 
                        mModules[i].getCurrentState()));

                setpointStates[i].optimize(mModules[i].getCurrentState().angle);
                setpointStates[i].cosineScale(mModules[i].getCurrentState().angle);
                optimizedSetpointStates[i] = mModules[i].setDesiredState(setpointStates[i]);
            }
        }

        // mPrevSetpointStates = optimizedSetpointStates;
        Telemetry.log("Drive/Swerve/Setpoints", unOptimizedSetpointStates);
        Telemetry.log("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
        Telemetry.log("Drive/Swerve/SetpointsChassisSpeeds", kKinematics.toChassisSpeeds(optimizedSetpointStates));
        Telemetry.log("Drive/Odometry/FieldSetpointChassisSpeed", ChassisSpeeds.fromRobotRelativeSpeeds(
            kKinematics.toChassisSpeeds(optimizedSetpointStates), mRobotRotation));
        Telemetry.log("Drive/Swerve/ModuleTorqueFF", moduleTorques);
    }

    /* Calculates DriveFeedforward based off state */
    public double calculateDriveFeedforward(SwerveModuleState[] unoptimizedSetpointStates, int i) {
        double driveAmps = (mUseChoreoFeedForward) 
            ? SwerveHelper.convertChoreoNewtonsToAmps(
                getModuleStates()[i], 
                unoptimizedSetpointStates[i],
                mPathPlanningFF, 
                i)
            : mDefaultFF.torqueCurrentsAmps()[i] 
                * SwerveHelper.ppFFScalar(
                    getModuleStates()[i], 
                    unoptimizedSetpointStates[i],
                    i);

        if(mFilterFeedForward) {
            driveAmps = SwerveHelper.lowPassFilter(
                mPrevDriveAmps[i], 
                driveAmps, 
                tDriveFFAggressiveness.get());
        }

        mPrevDriveAmps[i] = driveAmps;
        return driveAmps;
    }

    public void setFFModel(boolean pUseChoreoFeedForward, boolean pFilterFeedForward) {
        mUseChoreoFeedForward = pUseChoreoFeedForward;
        mFilterFeedForward = pFilterFeedForward;
    }

    public void setDriveFeedforwards(DriveFeedforwards ffs) {
        mPathPlanningFF = ffs;
    }

    ////////////// LOCALIZATION(MAINLY RESETING LOGIC) \\\\\\\\\\\\\\\\
    public void resetGyro() {
        /* Robot is usually facing the other way(relative to field) when doing cycles on red side, so gyro is reset to 180 */
        mRobotRotation = AllianceFlipUtil.shouldFlip() 
            ? Rotation2d.fromDegrees(180.0) 
            : Rotation2d.fromDegrees(0.0);

        mGyro.resetGyro(mRobotRotation);
    }

    public void setPose(Pose2d pose) {
        setPoses(pose, pose);
    }

    public void setPoses(Pose2d estimatorPose, Pose2d odometryPose) {
        mRobotRotation = estimatorPose.getRotation();
        mGyro.resetGyro(mRobotRotation);
        // Safe to pass in odometry poses because of the syncing
        // between gyro and pose estimator in reset gyro function
        mPoseEstimator.resetPosition(getRobotRotation(), getModulePositions(), estimatorPose);
        mOdometry.resetPosition(getRobotRotation(), getModulePositions(), odometryPose);
    }

    public void resetModulesEncoders() {
        for (int i = 0; i < 4; i++) mModules[i].resetAzimuthEncoder();
    }

    ///////////////////////// GETTERS \\\\\\\\\\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = mModules[i].getCurrentState();
        return states;
    }

    @AutoLogOutput(key = "Drive/Swerve/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = mModules[i].getCurrentPosition();
        return positions;
    }

    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getPoseEstimate() {
        return mPoseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Drive/Odometry/OdometryPose")
    public Pose2d getOdometryPose() {
        return mOdometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotation() {
        return mRobotRotation;
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotationVelocity() {
        return mGyroInputs.iYawVelocityPS;
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotChassisSpeeds")
    public ChassisSpeeds getRobotChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return mDesiredSpeeds;
    }

    public RobotConfig getPPRobotConfig() {
        return mRobotConfig;
    }

    public DriveFeedforwards getDriveFeedforwards() {
        return mPathPlanningFF;
    }

    public Module[] getModules() {
        return this.mModules;
    }
}