package frc.robot.systems.drive;

import static frc.robot.systems.drive.DriveConstants.kPPRotationPID;
import static frc.robot.systems.drive.DriveConstants.kPPTranslationPID;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controls.TurnPointFeedforward;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.GeomUtil;
import frc.lib.telemetry.Telemetry;
import frc.robot.systems.drive.controllers.HeadingController;
import frc.robot.systems.drive.controllers.HolonomicController;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.drive.controllers.LineController;
import frc.robot.systems.drive.controllers.ManualTeleopController;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.errors.DriveErrors;
import frc.robot.game.GameDriveManager;
import frc.robot.game.GameDriveManager.GameDriveStates;

public class DriveManager {
    public static enum DriveState {
        // TELEOP AND AUTON CONTROLS
        TELEOP,
        TELEOP_SNIPER,
        POV_SNIPER,
        HEADING_ALIGN,
        AUTON_HEADING_ALIGN,
        AUTO_ALIGN,
        LINE_ALIGN,
        AUTON,
        STOP,

        // TUNING
        DRIFT_TEST,
        LINEAR_TEST,
        SYSID_CHARACTERIZATION,
        WHEEL_CHARACTERIZATION
    }

    public Drive mDrive;

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

    private GameDriveManager mGameDriveManager;

    public DriveManager(Drive pDrive) {
        mDrive = pDrive;
        mGameDriveManager = new GameDriveManager(mDrive);
        mHeadingController.setHeadingGoal(mGoalRotationSup);
    }

    public ChassisSpeeds computeDesiredSpeeds() {
        mHeadingController.updateController();
        mAutoAlignController.updateControllers();
        mLineAlignController.updateControllers();

        ChassisSpeeds teleopSpeeds = mTeleopController.computeChassisSpeeds(
            mDrive.getPoseEstimate().getRotation(), 
            false, 
            true);
        ChassisSpeeds desiredSpeeds = teleopSpeeds;
        switch (mDriveState) {
            case TELEOP:
                break;
            case TELEOP_SNIPER:
                desiredSpeeds = mTeleopController.computeChassisSpeeds(
                   mDrive.getPoseEstimate().getRotation(), 
                   true, 
                   true);
                break;
            case POV_SNIPER:
                desiredSpeeds = mTeleopController.computeSniperPOVChassisSpeeds(
                   mDrive.getPoseEstimate().getRotation(), 
                   false);
                break;
            case AUTON_HEADING_ALIGN:
                desiredSpeeds = new ChassisSpeeds(
                    mDrive.getPPDesiredChassisSpeeds().vxMetersPerSecond, 
                    mDrive.getPPDesiredChassisSpeeds().vyMetersPerSecond,
                    mHeadingController.getSnapOutputRadians(mDrive.getPoseEstimate().getRotation()));
                break;
            case HEADING_ALIGN:
                desiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, 
                    teleopSpeeds.vyMetersPerSecond,
                    mHeadingController.getSnapOutputRadians(mDrive.getPoseEstimate().getRotation()));
                break;
            case AUTO_ALIGN:
                desiredSpeeds = mAutoAlignController.calculate(
                    mGoalPoseSup.get(), 
                    mDrive.getPoseEstimate());
                break;
            case LINE_ALIGN:
                desiredSpeeds = mLineAlignController.calculate(
                    teleopSpeeds, 
                    mGoalPoseSup.get(), 
                    mDrive.getPoseEstimate());
                break;
            case AUTON:
                desiredSpeeds = mDrive.getPPDesiredChassisSpeeds();
                break;
            case DRIFT_TEST:
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        Drive.tLinearTestSpeedMPS.get(), 
                        0.0, 
                        Math.toRadians(
                            Drive.tRotationDriftTestSpeedDeg.get())),
                    mDrive.getPoseEstimate().getRotation());
                break;
            case LINEAR_TEST:
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        Drive.tLinearTestSpeedMPS.get(),
                        0.0,
                        0.0), 
                    mDrive.getPoseEstimate().getRotation());
                break;
                /* Set by characterization commands in the CHARACTERIZATION header. Wheel characterization is currently unimplemented */
            case SYSID_CHARACTERIZATION:
            case WHEEL_CHARACTERIZATION:
                /* If null, then PID isn't set, so characterization can set motors w/o interruption */
                desiredSpeeds = null;
                break;
            case STOP:
                desiredSpeeds = new ChassisSpeeds();
                break;
            default:
                /* Defaults to Teleop control if no other cases are run*/
        }

        return desiredSpeeds;
    }

    ///////////////////////// STATE COMMANDS \\\\\\\\\\\\\\\\\\\\\\\\
    public void setDriveState(DriveState state) {
        mDriveState = state;
        switch (mDriveState) {
            case AUTON:
                mDrive.setFFModel(true, false);
                break;
            case AUTO_ALIGN, STOP:
                mDrive.setFFModel(false, false);
                break;
            default:
                mDrive.setFFModel(false, true);
        }
    } 

    public Command setDriveStateCommand(DriveState state) {
        return Commands.runOnce(() -> setDriveState(state), mDrive);
    }

    /* Set's state initially, and doesn't end till interruped by another drive command */
    public Command setDriveStateCommandContinued(DriveState state) {
        return new FunctionalCommand(() -> setDriveState(state), () -> {}, (interrupted) -> {}, () -> false, mDrive);
    }
    
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
        return setDriveStateCommand(DriveState.STOP);
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
            path, mDrive::getPoseEstimate, 
            mDrive::getRobotChassisSpeeds,
            (speeds, ff) -> {
                setDriveState(DriveState.AUTON);
                mDrive.setPPDesiredSpeeds(speeds);
                mDrive.setDriveFeedforwards(ff);
            }, 
            drivePID, 
            mDrive.getPPRobotConfig(), 
            () -> AllianceFlipUtil.shouldFlip(), 
            mDrive);
    }

    public Command getGameDriveCommand(GameDriveStates pGameDriveStates) {
        return mGameDriveManager.getSetGameDriveStateCmd(pGameDriveStates);
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of mDrive command
     * @param Goal strategy, based on where you're aligning
     * @param Constraint type, linear or on an axis
     */
    public Command setToGenericAutoAlign(Supplier<Pose2d> pGoalPoseSup, ConstraintType pConstraintType) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mAutoAlignController.setConstraintType(pConstraintType);
            mAutoAlignController.reset(
                mDrive.getPoseEstimate(), 
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    mDrive.getRobotChassisSpeeds(), 
                    mDrive.getPoseEstimate().getRotation()),
                mGoalPoseSup.get());
            }).andThen( setDriveStateCommandContinued( DriveState.AUTO_ALIGN ) );
    }

    public Command setToGenericLineAlign(Supplier<Pose2d> pGoalPoseSup, Supplier<Rotation2d> pAngle, DoubleSupplier pTelScal, BooleanSupplier pTelInv) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mLineAlignController.setControllerGoalSettings(
                pTelScal, 
                () -> pAngle.get().getTan(), 
                pTelInv);
            mLineAlignController.reset(
                mDrive.getPoseEstimate(), 
                mGoalPoseSup.get());
        }).andThen( setDriveStateCommandContinued( DriveState.LINE_ALIGN ) );
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of mDrive command
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
     * Reference GameDriveManager to use game-specific implementation of mDrive command
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
            if(!(headingState.equals(DriveState.AUTON_HEADING_ALIGN) || headingState.equals(DriveState.HEADING_ALIGN))) {
                Telemetry.reportIssue(new DriveErrors.WrongHeadingState());
            }
            mGoalRotationSup = pGoalRotation;
            mHeadingController.setHeadingGoal(mGoalRotationSup);
            mHeadingController.reset(
                mDrive.getPoseEstimate().getRotation(), 
                mDrive.getRobotRotationVelocity());
            mHeadingController.setTurnPointFF(pTurnPointFeedforward);
        }).andThen( setDriveStateCommandContinued( headingState ) );
    }

    public TurnPointFeedforward getDefaultTurnPointFF() {
        return new TurnPointFeedforward(
            () -> mDrive.getPoseEstimate(), 
            () -> mDrive.getDesiredChassisSpeeds(), 
            mGoalPoseSup, 
            () -> new ChassisSpeeds());
    }

    ///////////// SETTERS \\\\\\\\\\\\\
    public void acceptJoystickInputs(
            DoubleSupplier pXSupplier, DoubleSupplier pYSupplier,
            DoubleSupplier pThetaSupplier, Supplier<Rotation2d> pPOVSupplier) {
        mTeleopController.acceptJoystickInputs(pXSupplier, pYSupplier, pThetaSupplier, pPOVSupplier);
    }

    public Command setDriveProfile(DriverProfiles profile) {
        return new InstantCommand(() -> mTeleopController.updateTuneablesWithProfiles(profile));
    }

    ///////////// GETTERS \\\\\\\\\\\\\
    public BooleanSupplier waitUntilHeadingAlignFinishes() {
        return () -> mHeadingAlignTimeout.calculate(inHeadingTolerance());
    }

    public BooleanSupplier waitUntilAutoAlignFinishes() {
        return () -> mAutoAlignTimeout.calculate(mAutoAlignController.atGoal());
    }

    @AutoLogOutput(key = "Drive/Tolerance/HeadingController")
    public boolean inHeadingTolerance() {
        /* Accounts for angle wrapping issues with rotation 2D error */
        return GeomUtil.getSmallestChangeInRotation(
            mDrive.getPoseEstimate().getRotation(), 
            mGoalRotationSup.get()).getDegrees()
            < HeadingController.mToleranceDegrees.get();
    }

    public ManualTeleopController getTeleopController() {
        return mTeleopController;
    }

    public HolonomicController getAutoAlignController() {
        return mAutoAlignController;
    }

    public HeadingController getHeadingController() {
        return mHeadingController;
    }

    public LineController getLineAlignController() {
        return mLineAlignController;
    }
}
