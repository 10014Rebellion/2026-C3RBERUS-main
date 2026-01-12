package frc.robot.auton;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.drive.Drive;
import frc.robot.game.GameGoalPoseChooser.SIDE;
import frc.lib.math.AllianceFlipUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutonCommands extends SubsystemBase {
    public static enum AutoState {
        PreScoreCoral,
        ScoreCoral,
        PreIntakeCoral,
        IntakeCoral
    }

    private final Drive mRobotDrive;

    private final SendableChooser<Command> mAutoChooser;

    private Trigger mIR1;
    private Trigger mIR2;
    private Trigger mIR3;

    private Trigger mElevInIntakeWristExitRange;
    private Trigger mWristInIntakeEntryRange;
    private Trigger mWristInScoringRange;
    
    private Trigger mInAutoAlignRange;
    private Trigger mInDrivingScoringTolerance;
    private Trigger mInIntakeStartTolerance;

    public AutonCommands(Drive pRobotDrive) {
        this.mRobotDrive = pRobotDrive;

        mAutoChooser = new SendableChooser<>();

        mAutoChooser.setDefaultOption("Stationary", backUpAuton());
        tryToAddPathToChooser("String", nextScoreCoralPath("A1-Barge"));
    
        mIR1 = new Trigger(() ->  true);
        mIR2 = new Trigger(() ->  true);
        mIR3 = new Trigger(() ->  true);
    
        mElevInIntakeWristExitRange = new Trigger(() ->  true);
        mWristInIntakeEntryRange = new Trigger(() ->  true);
        mWristInScoringRange = new Trigger(() ->  true);
        
        mInAutoAlignRange = new Trigger(() ->  true);
        mInDrivingScoringTolerance = new Trigger(() ->  true);
        mInIntakeStartTolerance = new Trigger(() ->  true);
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public void tryToAddPathToChooser(String pPathName, Command... pCommands) {
        for(Command path : pCommands) {
            tryToAddPathToChooser(pPathName, new Runnable() {
                @Override
                public void run() {
                    mAutoChooser.addOption(pPathName, path);
                }
            });
        }
    }  
    
    /* Stops magic auton errors from occuring due to FMS or some BS I cook up */
    public void tryToAddPathToChooser(String pPathName, Runnable pPathAdding) {
        try {
            pPathAdding.run();
        } catch(Exception e) {
            mAutoChooser.addOption("Failed: "+pPathName, backUpAuton());
        }
    }

    public SendableChooser<Command> getmAutoChooser() {
        return mAutoChooser;
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public PathPlannerAuto firstPath(String pName, Rotation2d pStartingRotation, BooleanSupplier pConditionSupplier, Command pNextCommand, Command pNextAuto) {
        PathPlannerAuto firstAuto = new PathPlannerAuto(followFirstChoreoPath(pName, pStartingRotation));
        firstAuto.condition(pConditionSupplier).onTrue(pNextCommand.andThen(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(nextAutoChecker(pNextAuto)))));
        return firstAuto;
    }

    public PathPlannerAuto nextPath(String pName, BooleanSupplier pConditionSupplier, Command pNextCommand, Command pNextAuto) {
        PathPlannerAuto auto = new PathPlannerAuto(followChoreoPath(pName));
        auto.condition(pConditionSupplier).onTrue(pNextCommand.andThen(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(nextAutoChecker(pNextAuto)))));
        return auto;
    }

    public PathPlannerAuto nextPath(String pName, BooleanSupplier pConditionSupplier, Command pNextCommand, Command pNextAuto, PPHolonomicDriveController pPID) {
        PathPlannerAuto auto = new PathPlannerAuto(followChoreoPath(pName, pPID));
        auto.condition(pConditionSupplier).onTrue(pNextCommand.andThen(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(nextAutoChecker(pNextAuto)))));
        return auto;
    }

    public Command nextAutoChecker(Command pAuto) {
        return (pAuto == null) ? mRobotDrive.setToStop() : pAuto;
    }

    public Command backUpAuton() {
        return new InstantCommand();
    }

    public PathPlannerAuto nextScoreCoralPath(String pName) {
        PathPlannerAuto auto = new PathPlannerAuto(autoPlaceholder());

        auto.activePath(pName)
            .onTrue(followChoreoPath(pName))
            .onTrue(elevatorToPreScoreCommand())
            .onTrue(wristToPreScoreCommand())
            .onTrue(intakePivotToPreScoreCommand())
            .onTrue(intakeToPreScoreCommand())
            .onTrue(indexerToPreScoreCommand());

        auto.condition(mInAutoAlignRange)
            .onTrue(mRobotDrive.setToGenericAutoAlign(null, null));

        auto.condition(mInDrivingScoringTolerance)
            .onTrue(wristToScoreCommand());

        auto.condition(mInDrivingScoringTolerance.and(mWristInScoringRange))
            .onTrue(clawEjectCommand());

        auto.condition(mIR3.negate())
            .onTrue(clawHoldCommand())
            .onTrue(Commands.run(() -> auto.cancel()));

        return auto;
    }

    public PathPlannerAuto nextIntakeCoralPath(String pName) {
        PathPlannerAuto auto = new PathPlannerAuto(autoPlaceholder());

        auto.activePath(pName)
            .onTrue(followChoreoPath(pName));

        auto.condition(mInIntakeStartTolerance)
            .onTrue(elevatorToPreIntakeCommand())
            .onTrue(wristToPreIntakeCommand())
            .onTrue(intakePivotCoralCommand())
            .onTrue(
                new SequentialCommandGroup(
                    intakeCoralCommand(),
                    Commands.waitSeconds(1),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            setIntakeVoltCommand(12),
                            Commands.waitSeconds(0.1),
                            setIntakeVoltCommand(-12),
                            Commands.waitSeconds(0.1)
                        )
                    ).withTimeout(0.5)
                )
            )
            .onTrue(
                new SequentialCommandGroup(
                    intakeCoralCommand(),
                    Commands.waitSeconds(1),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            setIntakeVoltCommand(12),
                            Commands.waitSeconds(0.1),
                            setIntakeVoltCommand(-12),
                            Commands.waitSeconds(0.1)
                        )
                    ).withTimeout(0.5)
                )
            );

        auto.condition(mIR1)
            .onTrue(new SequentialCommandGroup(
                Commands.waitSeconds(1),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            setIntakeVoltCommand(12),
                            Commands.waitSeconds(0.1),
                            setIntakeVoltCommand(-12),
                            Commands.waitSeconds(0.1)
                        )
                    ).withTimeout(0.5)
                )
            )
            .onTrue(new SequentialCommandGroup(
                Commands.waitSeconds(1),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            setIntakeVoltCommand(12),
                            Commands.waitSeconds(0.1),
                            setIntakeVoltCommand(-12),
                            Commands.waitSeconds(0.1)
                        )
                    ).withTimeout(0.5)
                )
            );

        auto.condition(mIR2.and(mWristInIntakeEntryRange))
            .onTrue(clawIntakeCommand())
            .onTrue(elevatorToIntakeCommand());

        auto.condition(mIR3)
            .onTrue(clawHoldCommand())
            .onTrue(elevatorToIntakeCommand());

        auto.condition(mIR3.and(mElevInIntakeWristExitRange))
            .onTrue(Commands.run(() -> auto.cancel()));

        return auto;
    }

    ///////////////// SUPERSTRUCTURE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\
    public Command clawHoldCommand() {
        return new InstantCommand();
    }

    public Command clawEjectCommand() {
        return new InstantCommand();
    }

    public Command clawIntakeCommand() {
        return new InstantCommand();
    }
    
    public Command elevatorToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command wristToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command intakeToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command intakePivotToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command indexerToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command clawToPreScoreCommand() {
        return new InstantCommand();
    }

    public Command wristToScoreCommand() {
        return new PrintCommand("Score Coral");
    }

    public Command elevatorToPreIntakeCommand() {
        return new InstantCommand();
    }

    public Command wristToPreIntakeCommand() {
        return new InstantCommand();
    }

    public Command intakeCoralCommand() {
        return new PrintCommand("Intake Coral");
    }

    public Command setIntakeVoltCommand(double volts) {
        return new PrintCommand("Intake Coral Volts");
    }

    public Command intakePivotCoralCommand() {
        return new PrintCommand("Intake Coral");
    }

    public Command indexCoralCommand() {
        return new PrintCommand("Index Coral");
    }

    public Command setIndexVoltCommand(double volts) {
        return new PrintCommand("Intake Coral Volts");
    }

    public Command elevatorToIntakeCommand() {
        return new InstantCommand();
    }

    public Command scoreAlgaeCommand() {
        return new PrintCommand("Score Algae");
    }

    public Command intakeAlgaeCommand() {
        return new PrintCommand("Intake Algae");
    }

    public BooleanSupplier getIntakeIR1HasPiece() {
        return () -> false;
    }

    public BooleanSupplier getIntakeIR2HasPiece() {
        return () -> false;
    }
    public BooleanSupplier getClawIR3HasPiece() {
        return () -> false;
    }

    public BooleanSupplier getIntakePivotAtGoal() {
        return () -> false;
    }

    public BooleanSupplier getElevatorAtGoal() {
        return () -> false;
    }

    public BooleanSupplier getWristAtGoal() {
        return () -> false;
    } 

    ///////////////// PATH CREATION LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command followFirstChoreoPath(String pathName, Rotation2d startingRotation) {
        PathPlannerPath path = getTraj(pathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                mRobotDrive.setPose(AllianceFlipUtil.apply(new Pose2d(path.getPathPoses().get(0).getTranslation(), startingRotation)));
            }), 
            mRobotDrive.customFollowPathComamnd(path).withTimeout(totalTimeSeconds), 
            mRobotDrive.setToStop());
    }

    public Command followChoreoPath(String pathName) {
        PathPlannerPath path = getTraj(pathName).get();
        path.getIdealTrajectory(Drive.mRobotConfig);
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();
        return 
                mRobotDrive.customFollowPathComamnd(path).withTimeout(totalTimeSeconds).andThen(
                mRobotDrive.setToStop());
    }

    public Command followChoreoPath(String pathName, PPHolonomicDriveController PID) {
        PathPlannerPath path = getTraj(pathName).get();
        path.getIdealTrajectory(Drive.mRobotConfig);
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();
        return 
        mRobotDrive.customFollowPathComamnd(path, PID).withTimeout(totalTimeSeconds).andThen(
            mRobotDrive.setToStop());
    }

    public Optional<PathPlannerPath> getTraj(String pathName) {
        try {
            return Optional.of(PathPlannerPath.fromChoreoTrajectory(pathName));
        } catch(Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public SIDE getSide(String name){
        String n = name.substring(6, 7);
        return (n.equals("L")) ? SIDE.LEFT : SIDE.RIGHT;
    }

    public FunctionalCommand autoPlaceholder() {
        return new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {}, () -> false, this);
    }
}