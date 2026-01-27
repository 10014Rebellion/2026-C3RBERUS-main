// REBELLION 10014

package frc.robot.systems.drive;

import static frc.robot.systems.drive.DriveConstants.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controls.TurnPointFeedforward;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.GeomUtil;
import frc.lib.optimizations.PPRobotConfigLoader;
import frc.lib.pathplanner.AzimuthFeedForward;
import frc.lib.pathplanner.SwerveSetpoint;
import frc.lib.pathplanner.SwerveSetpointGenerator;
import frc.lib.swerve.SwerveUtils;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.game.GameDriveManager;
import frc.robot.game.GameDriveManager.GameDriveStates;
import frc.robot.systems.drive.controllers.HeadingController;
import frc.robot.systems.drive.controllers.HolonomicController;
import frc.robot.systems.drive.controllers.LineController;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.drive.controllers.ManualTeleopController;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroInputsAutoLogged;
import frc.robot.systems.drive.modules.Module;
import frc.robot.systems.apriltag.AprilTag;
import frc.robot.systems.apriltag.AprilTag.VisionObservation;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    public static enum DriveState {
        // TELEOP AND AUTON CONTROLS
        TELEOP,
        TELEOP_SNIPER,
        POV_SNIPER,
        HEADING_ALIGN,
        AUTON_HEADING_ALIGN,
        AUTO_ALIGN,
        LINE_ALIGN, // TODO: Implement this.
        AUTON,
        STOP,

        // TUNING
        DRIFT_TEST,
        LINEAR_TEST,
        SYSID_CHARACTERIZATION,
        WHEEL_CHARACTERIZATION
    }

    private final Module[] mModules;
    private final GyroIO mGyro;
    private final GyroInputsAutoLogged mGyroInputs = new GyroInputsAutoLogged();
    private final AprilTag mVision;

    private Rotation2d mRobotRotation;
    private final SwerveDriveOdometry mOdometry;
    private final SwerveDrivePoseEstimator mPoseEstimator;
    private final Field2d mField = new Field2d();
    private final Debouncer mSkidFactorDebouncer = new Debouncer(0.25, DebounceType.kFalling);
    private final Debouncer mCollisionDebouncer = new Debouncer(0.25, DebounceType.kFalling);

    public static RobotConfig mRobotConfig;
    private final SwerveSetpointGenerator mSetpointGenerator;
    private SwerveSetpoint mPreviousSetpoint =
            new SwerveSetpoint(new ChassisSpeeds(), SwerveUtils.zeroStates(), DriveFeedforwards.zeros(4), AzimuthFeedForward.zeros());

    private ChassisSpeeds mDesiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds mPPDesiredSpeeds = new ChassisSpeeds();
    private DriveFeedforwards mPathPlanningFF = DriveFeedforwards.zeros(4);
    private final PathConstraints mDriveConstraints = DriveConstants.kAutoConstraints;

    private SwerveModuleState[] mPrevSetpointStates = SwerveUtils.zeroStates();
    private SwerveModulePosition[] mPrevPositions = SwerveUtils.zeroPositions();
    private double[] mPrevDriveAmps = new double[] {0.0, 0.0, 0.0, 0.0};

    private final boolean kUseGenerator = true;

    @AutoLogOutput(key = "Drive/State")
    private DriveState mDriveState = DriveState.TELEOP;

    private final ManualTeleopController mTeleopController = new ManualTeleopController();

    private final HeadingController mHeadingController = new HeadingController(TurnPointFeedforward.zeroTurnPointFF());

    private Supplier<Rotation2d> mGoalRotationSup = () -> new Rotation2d();
    private final Debouncer mHeadingAlignTimeout = new Debouncer(0.1, DebounceType.kRising);

    private final HolonomicController mAutoAlignController = new HolonomicController();
    private final LineController mLineAlignController = new LineController(
        () -> 0.0, () -> 1.0, () -> false);

    private Supplier<Pose2d> mGoalPoseSup = () -> new Pose2d();
    private final Debouncer mAutoAlignTimeout = new Debouncer(0.1, DebounceType.kRising);

    private final GameDriveManager mGameDriveManager = new GameDriveManager(this);

    public static final LoggedTunableNumber tDriftRate = new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);
    public static final LoggedTunableNumber tRotationDriftTestSpeedDeg = new LoggedTunableNumber("Drive/DriftRotationTestDeg", 360);
    public static final LoggedTunableNumber tLinearTestSpeedMPS = new LoggedTunableNumber("Drive/LinearTestMPS", 4.5);
    public static final LoggedTunableNumber tAzimuthCharacterizationVoltage = new LoggedTunableNumber("Drive/AzimuthCharacterizationVoltage", 0);
    public static final LoggedTunableNumber tDriveFFAggressiveness = new LoggedTunableNumber("Drive/Teleop/DriveFFAggressiveness", kDriveFFAggressivness);
    
    public Drive(Module[] modules, GyroIO gyro, AprilTag vision) {
        this.mModules = modules;
        this.mGyro = gyro;
        this.mVision = vision;

        mRobotRotation = mGyroInputs.iYawPosition;

        mOdometry = new SwerveDriveOdometry(kKinematics, getRobotRotation(), getModulePositions());
        mPoseEstimator = new SwerveDrivePoseEstimator(kKinematics, getRobotRotation(), getModulePositions(), new Pose2d());

        mRobotConfig = PPRobotConfigLoader.load();
        mSetpointGenerator = new SwerveSetpointGenerator(mRobotConfig, kMaxAzimuthAngularRadiansPS);

        PhoenixOdometryThread.getInstance().start();

        AutoBuilder.configure(
            this::getPoseEstimate, this::setPose,this::getRobotChassisSpeeds,
            (speeds, ff) -> {
                mDriveState = DriveState.AUTON;
                mPPDesiredSpeeds = speeds;
                mPathPlanningFF = ff;
            },
            new PPHolonomicDriveController(kPPTranslationPID, kPPRotationPID),
            mRobotConfig, () -> AllianceFlipUtil.shouldFlip(), this);

        SwerveUtils.setUpPathPlanner();
        SmartDashboard.putData(mField);
        mHeadingController.setHeadingGoal(mGoalRotationSup);
    }

    ///// ENTRY POINT TO THE DRIVE \\\\\
    @Override
    public void periodic() {
        updateSensorsAndOdometry();
        runSwerve(computeDesiredSpeeds());
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
        double[] sampleTimestamps =
            mModules[0].getOdometryTimeStamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        // System.out.println("\n\n\n\n\n\n\n\n"+sampleCount+"\n\n\n\n\n\n\n\n\n\n");
        // Telemetry.reportException(new Exception("SAMPLE COUNT:" + sampleCount));
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = SwerveUtils.zeroPositions();
            SwerveModulePosition[] moduleDeltas = SwerveUtils.zeroPositions();
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                // System.out.println("\n\n\n\n\n\n\n\n"+mModules[moduleIndex].getOdometryPositions().length+"\n\n\n\n\n\n\n\n\n\n");
                modulePositions[moduleIndex] = mModules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - mPrevPositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
                mPrevPositions[moduleIndex] = modulePositions[moduleIndex];
            }

            Twist2d robotTwist = kKinematics.toTwist2d(moduleDeltas);
            if(kSkidRatioCap < SwerveUtils.skidRatio(GeomUtil.toChassisSpeeds(robotTwist, 1.0 / kOdometryFrequency))) skidCount++;

            // Update gyro angle
            if (mGyroInputs.iConnected) {
                // Use the real gyro angle
                mRobotRotation = mGyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kKinematics.toTwist2d(moduleDeltas);
                mRobotRotation = mRobotRotation.plus(new Rotation2d(twist.dtheta));
            }

            mPoseEstimator.updateWithTime(sampleTimestamps[i], mRobotRotation, modulePositions);
        }

        mOdometry.update(mRobotRotation, getModulePositions());

        double skidFactor = mSkidFactorDebouncer.calculate(skidCount > 0) ? skidCount * kSkidScalar : 0;
        double gyroFactor = mCollisionDebouncer.calculate(mGyro.getAccMagG() > kCollisionCapG) ? kCollisionScalar : 1.0;
        
        double visionFactor = skidFactor + gyroFactor;

        Logger.recordOutput("Drive/Odometry/SkidFactor", skidFactor);
        Logger.recordOutput("Drive/Odometry/SkidCount", skidCount);
        Logger.recordOutput("Drive/Odometry/GyroFactor", gyroFactor);
        Logger.recordOutput("Drive/Odometry/VisionFactor", visionFactor);
        
        /* VISION */
        mVision.periodic(mPoseEstimator.getEstimatedPosition(), mOdometry.getPoseMeters());
        VisionObservation[] observations = mVision.getVisionObservations();
        for (VisionObservation observation : observations) {
            if (observation.hasObserved()) mPoseEstimator.addVisionMeasurement(
                observation.pose(), observation.timeStamp(), 
                observation.stdDevs().times(1.0 / visionFactor));

            Telemetry.logVisionObservationStdDevs(observation);
        }

        mField.setRobotPose(getPoseEstimate());
    }

    private ChassisSpeeds computeDesiredSpeeds() {
        mHeadingController.updateController();
        mAutoAlignController.updateControllers();
        mLineAlignController.updateControllers();

        ChassisSpeeds teleopSpeeds = mTeleopController.computeChassisSpeeds(
            getPoseEstimate().getRotation(), false, true);
        switch (mDriveState) {
            case TELEOP:
                mDesiredSpeeds = teleopSpeeds;
                break;
            case TELEOP_SNIPER:
                mDesiredSpeeds = mTeleopController.computeChassisSpeeds(
                    getPoseEstimate().getRotation(), true, true);
                break;
            case POV_SNIPER:
                mDesiredSpeeds = mTeleopController.computeSniperPOVChassisSpeeds(
                    getPoseEstimate().getRotation(), false);
                break;
            case AUTON_HEADING_ALIGN:
                mDesiredSpeeds = new ChassisSpeeds(
                    mPPDesiredSpeeds.vxMetersPerSecond, mPPDesiredSpeeds.vyMetersPerSecond,
                    mHeadingController.getSnapOutputRadians(getPoseEstimate().getRotation()));
                break;
            case HEADING_ALIGN:
                mDesiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond,
                    mHeadingController.getSnapOutputRadians(getPoseEstimate().getRotation()));
                break;
            case AUTO_ALIGN:
                mDesiredSpeeds = mAutoAlignController.calculate(mGoalPoseSup.get(), getPoseEstimate());
                break;
            case LINE_ALIGN:
                mDesiredSpeeds = mLineAlignController.calculate(teleopSpeeds, mGoalPoseSup.get(), getPoseEstimate());
                break;
            case AUTON:
                mDesiredSpeeds = mPPDesiredSpeeds;
                break;
            case DRIFT_TEST:
                mDesiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                    tLinearTestSpeedMPS.get(), 0.0, Math.toRadians(tRotationDriftTestSpeedDeg.get())),
                    mRobotRotation);
                break;
            case LINEAR_TEST:
                mDesiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                    tLinearTestSpeedMPS.get(), 0.0, 0.0), mRobotRotation);
                break;
                /* Set by characterization commands in the CHARACTERIZATION header. Wheel characterization is currently unimplemented */
            case SYSID_CHARACTERIZATION:
            case WHEEL_CHARACTERIZATION:
                /* If null, then PID isn't set, so characterization can set motors w/o interruption */
                mDesiredSpeeds = null;
                break;
            case STOP:
                mDesiredSpeeds = new ChassisSpeeds();
                break;
            default:
                /* Defaults to Teleop control if no other cases are run*/
        }

        return mDesiredSpeeds;
    }

        ////////////// CHASSIS SPEED TO MODULES \\\\\\\\\\\\\\\\
    /* Sets the desired swerve module states to the robot */
    public void runSwerve(ChassisSpeeds speeds) {
        if(speeds == null) return;
        mDesiredSpeeds = SwerveUtils.discretize(speeds, tDriftRate.get());

        /* Logs all the possible drive states, great for debugging */
        SwerveUtils.logPossibleDriveStates(
                kDoExtraLogging, mDesiredSpeeds, getModuleStates(), mPreviousSetpoint, mRobotRotation);

        SwerveModuleState[] unOptimizedSetpointStates = new SwerveModuleState[4];
        SwerveModuleState[] setpointStates = kKinematics.toSwerveModuleStates(mDesiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeedMPS);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        mPreviousSetpoint = mSetpointGenerator.generateSetpoint(
            mPreviousSetpoint, mDesiredSpeeds, (DriverStation.isTeleop()) ? mDriveConstraints : kAutoConstraints, 0.02);

        /* Only for logging purposes */
        SwerveModuleState[] moduleTorques = SwerveUtils.zeroStates();

        // Telemetry.log("Drive/Odometry/generatedFieldSpeeds",
        // ChassisSpeeds.fromRobotRelativeSpeeds(previousSetpoint.robotRelativeSpeeds(), robotRotation));
        for (int i = 0; i < 4; i++) {
            if (kUseGenerator) {
                /* Logs the drive feedforward stuff */
                SwerveUtils.logDriveFeedforward(mPreviousSetpoint.feedforwards(), i);

                setpointStates[i] = new SwerveModuleState(
                    mPreviousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    /* setpointAngle = currentAngle if the speed is less than 0.01 */
                    SwerveUtils.removeAzimuthJitter(
                        mPreviousSetpoint.moduleStates()[i], mModules[i].getCurrentState()));

                unOptimizedSetpointStates[i] = SwerveUtils.copyState(setpointStates[i]);
                setpointStates[i].optimize(mModules[i].getCurrentState().angle);

                /* Feedforward cases based on driveState */
                double driveAmps = calculateDriveFeedforward(
                    mPreviousSetpoint, mModules[i].getCurrentState(),
                    unOptimizedSetpointStates[i], setpointStates[i], i);
                double desiredAzimuthVelocityRadPS = 
                    mPreviousSetpoint.azimuthFeedforwards().azimuthSpeedRadiansPS()[i];

                // Multiplies by cos(angleError) to stop the drive from going in the wrong direction
                setpointStates[i].cosineScale(mModules[i].getCurrentState().angle);

                optimizedSetpointStates[i] = mModules[i].setDesiredStateWithAmpFF(setpointStates[i], driveAmps, desiredAzimuthVelocityRadPS);

                Logger.recordOutput("Drive/DesiredAzimuthRotationSpeed"+mModules[i].getModuleName(), desiredAzimuthVelocityRadPS);
                /* Normalized for logging */
                moduleTorques[i] = new SwerveModuleState(
                    (driveAmps * kMaxLinearSpeedMPS / kDriveFOCAmpLimit), optimizedSetpointStates[i].angle);
            } else {
                setpointStates[i] = new SwerveModuleState(setpointStates[i].speedMetersPerSecond,
                    SwerveUtils.removeAzimuthJitter(setpointStates[i], mModules[i].getCurrentState()));

                setpointStates[i].optimize(mModules[i].getCurrentState().angle);
                setpointStates[i].cosineScale(mModules[i].getCurrentState().angle);
                optimizedSetpointStates[i] = mModules[i].setDesiredState(setpointStates[i]);
            }
        }

        mPrevSetpointStates = optimizedSetpointStates;

        Telemetry.log("Drive/Swerve/Setpoints", unOptimizedSetpointStates);
        Telemetry.log("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
        Telemetry.log("Drive/Swerve/SetpointsChassisSpeeds", kKinematics.toChassisSpeeds(optimizedSetpointStates));
        Telemetry.log("Drive/Odometry/FieldSetpointChassisSpeed", ChassisSpeeds.fromRobotRelativeSpeeds(
            kKinematics.toChassisSpeeds(optimizedSetpointStates), mRobotRotation));
        Telemetry.log("Drive/Swerve/ModuleTorqueFF", moduleTorques);
    }

    /* Calculates DriveFeedforward based off state */
    public double calculateDriveFeedforward(
            SwerveSetpoint setpoint, SwerveModuleState currentState,
            SwerveModuleState unoptimizedState, SwerveModuleState optimizedState, int i) {
        double driveAmps = 0.0;
        switch (mDriveState) {
            /* No need to optimize for Choreo, as it handles it under the hood */
            case AUTON:
                driveAmps = SwerveUtils.convertChoreoNewtonsToAmps(currentState, mPathPlanningFF, i);
                break;
            case AUTO_ALIGN:
            default:
                driveAmps = setpoint.feedforwards().torqueCurrentsAmps()[i];
        }

        driveAmps = SwerveUtils.optimizeTorque(unoptimizedState, optimizedState,
            setpoint.feedforwards().torqueCurrentsAmps()[i], currentState, i);

        if(!mDriveState.equals(DriveState.AUTON) && !mDriveState.equals(DriveState.AUTO_ALIGN)) 
            driveAmps = SwerveUtils.lowPassFilter(mPrevDriveAmps[i], driveAmps, tDriveFFAggressiveness.get());

        mPrevDriveAmps[i] = driveAmps;
        return driveAmps;
    }

    ///////////////////////// STATE SETTING \\\\\\\\\\\\\\\\\\\\\\\\
    public Command setToTeleop() {
        return setDriveStateCommandContinued(DriveState.TELEOP);
    }

    public Command setToTeleopSniper() {
        return setDriveStateCommandContinued(DriveState.TELEOP_SNIPER);
    }

    public Command setToPOVSniper() {
        return setDriveStateCommandContinued(DriveState.POV_SNIPER);
    }

    public Command setToStop() {
        return setDriveStateCommandContinued(DriveState.STOP);
    }

    public Command setToDriftTest() {
        return setDriveStateCommandContinued(DriveState.DRIFT_TEST);
    }

    public Command setToLinearTest() {
        return setDriveStateCommandContinued(DriveState.LINEAR_TEST);
    }

    public Command setToSysIDCharacterization() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION);
    }

    public Command setToWheelCharacterization() {
        return setDriveStateCommand(DriveState.WHEEL_CHARACTERIZATION);
    }

    public Command customFollowPathCommand(PathPlannerPath path) {
        return customFollowPathCommand(path, new PPHolonomicDriveController(kPPTranslationPID, kPPRotationPID));
    }

    public Command customFollowPathCommand(PathPlannerPath path, PPHolonomicDriveController drivePID) {
        return new FollowPathCommand(
            path, this::getPoseEstimate, this::getRobotChassisSpeeds,
            (speeds, ff) -> {
                setDriveState(DriveState.AUTON);
                mPPDesiredSpeeds = speeds;
                mPathPlanningFF = ff;
            }, drivePID, 
            mRobotConfig, () -> AllianceFlipUtil.shouldFlip(), this);
    }

    public Command getGameDriveCommand(GameDriveStates pGameDriveStates) {
        return mGameDriveManager.getSetGameDriveStateCmd(pGameDriveStates);
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of this command
     * @param Goal strategy, based on where you're aligning
     * @param Constraint type, linear or on an axis
     */
    public Command setToGenericAutoAlign(Supplier<Pose2d> pGoalPoseSup, ConstraintType pConstraintType) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mAutoAlignController.setConstraintType(pConstraintType);
            mAutoAlignController.reset(
                getPoseEstimate(), ChassisSpeeds.fromRobotRelativeSpeeds(
                    getRobotChassisSpeeds(), getPoseEstimate().getRotation()),
                mGoalPoseSup.get());
            }).andThen( setDriveStateCommandContinued( DriveState.AUTO_ALIGN ) );
    }

    public Command setToGenericLineAlign(Supplier<Pose2d> pGoalPoseSup, Supplier<Rotation2d> pAngle, DoubleSupplier pTelScal, BooleanSupplier pTelInv) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mLineAlignController.setControllerGoalSettings(pTelScal, () -> pAngle.get().getTan(), pTelInv);
            mLineAlignController.reset(getPoseEstimate(), mGoalPoseSup.get());
        }).andThen( setDriveStateCommandContinued( DriveState.LINE_ALIGN ) );
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of this command
     * @param The desired rotation
     * @param Turn feedforward
     */
    public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward) {
        return setToGenericHeadingAlign( pGoalRotation, pTurnPointFeedforward, DriveState.HEADING_ALIGN );
    }

    /* Accounts for velocity of drive when turning */
    public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation) {
        return setToGenericHeadingAlign( pGoalRotation, getDefaultTurnPointFF() );
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of this command
     * @param The desired rotation
     * @param Turn feedforward
     */
    public Command setToGenericHeadingAlignAuton(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward) {
        return setToGenericHeadingAlign( pGoalRotation, pTurnPointFeedforward, DriveState.AUTON_HEADING_ALIGN );
    }

    /* Accounts for velocity of drive when turning */
    public Command setToGenericHeadingAlignAuton(Supplier<Rotation2d> pGoalRotation) {
        return setToGenericHeadingAlignAuton( pGoalRotation, getDefaultTurnPointFF() );
    }

    public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward, DriveState headingState) {
        return new InstantCommand(() -> {
            mGoalRotationSup = pGoalRotation;
            mHeadingController.setHeadingGoal(mGoalRotationSup);
            mHeadingController.reset(getPoseEstimate().getRotation(), mGyroInputs.iYawVelocityPS);
            mHeadingController.setTurnPointFF(pTurnPointFeedforward);
        }).andThen( setDriveStateCommandContinued( headingState ) );
    }

    public TurnPointFeedforward getDefaultTurnPointFF() {
        return new TurnPointFeedforward(
            mPoseEstimator::getEstimatedPosition, this::getDesiredChassisSpeeds, 
            mGoalPoseSup, () -> new ChassisSpeeds());
    }

    ////// BASE STATES \\\\\\
    private Command setDriveStateCommand(DriveState state) {
        return Commands.runOnce(() -> setDriveState(state), this);
    }

    /* Set's state initially, and doesn't end till interruped by another drive command */
    private Command setDriveStateCommandContinued(DriveState state) {
        return new FunctionalCommand(() -> setDriveState(state), () -> {}, (interrupted) -> {}, () -> false, this);
    }

    /* Sets the drive state used in periodic(), and handles init condtions like resetting PID controllers */
    private void setDriveState(DriveState state) {
        mDriveState = state;
    }

    ////////////// LOCALIZATION(MAINLY RESETING LOGIC) \\\\\\\\\\\\\\\\
    public void resetGyro() {
        /* Robot is usually facing the other way(relative to field) when doing cycles on red side, so gyro is reset to 180 */
        mRobotRotation = AllianceFlipUtil.shouldFlip() ? Rotation2d.fromDegrees(180.0) : Rotation2d.fromDegrees(0.0);
        mGyro.resetGyro(mRobotRotation);
        setPose(new Pose2d(new Translation2d(), mRobotRotation));
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

    /* Drive setters */
    public void acceptJoystickInputs(
            DoubleSupplier pXSupplier, DoubleSupplier pYSupplier,
            DoubleSupplier pThetaSupplier, Supplier<Rotation2d> pPOVSupplier) {
        mTeleopController.acceptJoystickInputs(pXSupplier, pYSupplier, pThetaSupplier, pPOVSupplier);
    }

    public Command setDriveProfile(DriverProfiles profile) {
        return new InstantCommand(() -> mTeleopController.updateTuneablesWithProfiles(profile));
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

    @AutoLogOutput(key = "Drive/Odometry/RobotChassisSpeeds")
    public ChassisSpeeds getRobotChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return mDesiredSpeeds;
    }

    public BooleanSupplier waitUntilHeadingAlignFinishes() {
        return () -> mHeadingAlignTimeout.calculate(inHeadingTolerance());
    }

    public BooleanSupplier waitUntilAutoAlignFinishes() {
        return () -> mAutoAlignTimeout.calculate(mAutoAlignController.atGoal());
    }

    @AutoLogOutput(key = "Drive/Tolerance/HeadingController")
    public boolean inHeadingTolerance() {
        /* Accounts for angle wrapping issues with rotation 2D error */
        return GeomUtil.getSmallestChangeInRotation(mRobotRotation, mGoalRotationSup.get()).getDegrees()
            < HeadingController.mToleranceDegrees.get();
    }

    public Module[] getModules() {
        return this.mModules;
    }
}
