package frc.robot.systems.auton;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.lib.telemetry.Telemetry;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoEvent;
import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.conveyor.ConveyorSS.ConveyorState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.Shooter;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelState;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.lib.math.AllianceFlipUtil;

public class AutonCommands extends SubsystemBase {
    private final Drive mRobotDrive;
    private final Intake mIntake;
    private final Shooter mShooter;
    private final ConveyorSS mConveyorSS;

    private final SendableChooser<Supplier<Command>> mAutoChooser;
    private final LoggedDashboardChooser<Supplier<Command>> mAutoChooserLogged;

    public AutonCommands(Drive pRobotDrive, Intake pIntake, ConveyorSS mConveyor, Shooter pShooter) {
        this.mRobotDrive = pRobotDrive;
        this.mIntake = pIntake;
        this.mShooter = pShooter;
        this.mConveyorSS = mConveyor;

        mAutoChooser = new SendableChooser<>();

        mAutoChooser.setDefaultOption("Stationary", () -> backUpAuton());
        tryToAddPathToChooser("FirstTestPath", () -> firstPathTest("FirstPathTest", "FirstPath"));
        tryToAddPathToChooser("FirstAuto", () -> autoTest("FirstAuto","FirstPath", "SecondPath"));
        tryToAddPathToChooser("LeftScoreAndClimb", () -> leftScoreAndClimb());
        
        mAutoChooserLogged = new LoggedDashboardChooser<>("Autos", mAutoChooser);
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command backUpAuton() {
        return new InstantCommand();
    }

    public Command leftScoreAndClimb() {
        AutoEvent auto = new AutoEvent("LeftScoreAndClimb", this);
        Trigger autoActivted = auto.getIsRunningTrigger();

        String path1Name = "ITl_BN";

        SequentialEndingCommandGroup autoPath1 = followChoreoPath(path1Name, true);

        Trigger isPath1Running = auto.loggedCondition(path1Name+"/isRunning", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(path1Name+"/hasEnded", () -> autoPath1.hasEnded(), true);
        Trigger deployIntake = auto.loggedCondition(path1Name+"/deployIntake", () -> false, true);

        String path2Name = "P1BN_BS";

        SequentialEndingCommandGroup autoPath2 = followChoreoPath(path2Name, false);

        Trigger isPath2Running = auto.loggedCondition(path2Name+"/isRunning", () -> autoPath2.isRunning(), true);
        Trigger hasPath2Ended = auto.loggedCondition(path2Name+"/hasEnded", () -> autoPath2.hasEnded(), true);

        String path3Name = "BL_LC";

        SequentialEndingCommandGroup autoPath3 = followChoreoPath(path3Name, false);
        SequentialEndingCommandGroup shootAtEndCommand = new SequentialEndingCommandGroup(shotIndexCommand());
        SequentialEndingCommandGroup climbAtEndCommand = new SequentialEndingCommandGroup(shotIndexCommand());

        Trigger isPath3Running = auto.loggedCondition(path3Name+"/isRunning", () -> autoPath3.isRunning(), true);
        Trigger hasPath3Ended = auto.loggedCondition(path3Name+"/hasEnded", () -> autoPath3.hasEnded(), true);

        Trigger doneShootingAtEnd = auto.loggedCondition(path3Name+"/doneShootingAtEnd", () -> shootAtEndCommand.hasEnded(), true);
        Trigger doneClimingAtEnd = auto.loggedCondition(path3Name+"/doneClimbingAtEnd", () -> climbAtEndCommand.hasEnded(), true);

        autoActivted
            .onTrue(autoPath1);

        isPath1Running.and(deployIntake)
            .onTrue(deployIntakeCommand());

        hasPath1Ended
            .onTrue(autoPath2);

        hasPath2Ended
            .onTrue(autoPath3);

        isPath3Running
            .onTrue(spinFlywheelsCommand());

        hasPath3Ended
            .onTrue(shotIndexCommand());

        doneShootingAtEnd
            .onTrue(climbCommand());
    
        doneClimingAtEnd
            .onTrue(endAuto(auto));


        return auto;
    }

    public Command firstPathTest(String pAutoName, String pName) {
        AutoEvent auto = new AutoEvent(pAutoName, this);
        SequentialEndingCommandGroup autoPath1 = followChoreoPath(pName, true);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger isPath1Running = auto.loggedCondition(pName+"IsFinished", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(pName+"IsFinished", () -> autoPath1.hasEnded(), true);

        autoActivted
            .onTrue(autoPath1);

        hasPath1Ended
            .onTrue(Commands.runOnce(() -> auto.cancel()));

        return auto;
    }

    public Command autoTest(String pAutoName, String pName1, String pName2) {
        AutoEvent auto = new AutoEvent(pAutoName, this);
        SequentialEndingCommandGroup autoPath1 = followChoreoPath(pName1, true);
        SequentialEndingCommandGroup autoPath2 = followChoreoPath(pName2, false);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger isPath1Running = auto.loggedCondition(pName1+"IsRunning", autoPath1::isRunning, true);
        Trigger hasPath1Ended = auto.loggedCondition(pName1+"HasEnded", autoPath1::hasEnded, true);

        Trigger isPath2Running = auto.loggedCondition(pName2+"IsRunning", autoPath2::isRunning, true);
        Trigger hasPath2Ended = auto.loggedCondition(pName2+"HasEnded", autoPath2::hasEnded, true);

        Trigger inScoringRange = inScoringRange(auto);
        Trigger inIntakeRange = inIntakeRange(auto);
        Trigger flywheelsReady = flywheelsReady(auto);
        Trigger hoodReady = hoodReady(auto);

        autoActivted
            .onTrue(autoPath1);
        
        hasPath1Ended
            .onTrue(autoPath2);

        auto.loggedCondition(pName2+"isIntaking", isPath2Running.and(inIntakeRange), true)
            .onTrue(bindexCommand())
            .onTrue(intakeCommand());

        hasPath2Ended
            .onTrue(Commands.runOnce(() -> auto.cancel()));

        return auto;
    }

    ///////////////// SUPERSTRUCTURE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\
    public Command intakeCommand() {
        return mIntake.setPivotStateCmd(IntakePivotState.INTAKE).alongWith(mIntake.setRollerStateCmd(IntakeRollerState.INTAKE));
    }

    public Command deployIntakeCommand() {
        return mIntake.setPivotStateCmd(IntakePivotState.INTAKE);
    }

    public Command bindexCommand() {
        return new InstantCommand();
    }

    public Command shotIndexCommand() {
        return mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE).alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE));
    }
    
    public Command spinFlywheelsCommand() {
        return mShooter.setFlywheelStateCmd(FlywheelState.SHOOT_CLOSE);
    }

    public Command turnToHubCommand() {
        return mRobotDrive.getDriveManager().setToGenericHeadingAlign(() -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), mRobotDrive.getDriveManager().getDefaultTurnPointFF());
    }

    public Command climbCommand() {
        return new InstantCommand();
    }

    public Command endAuto(AutoEvent auto) {
        return Commands.runOnce(() -> auto.cancel());
    }

    public BooleanSupplier inScoringRange() {
        return () -> false;
    }

    public BooleanSupplier flywheelsReady() {
        return () -> false;
    }

    public BooleanSupplier hoodReady() {
        return () -> false;
    }

    public BooleanSupplier inIntakeRange() {
        return () -> false;
    }

    public Trigger inScoringRange(AutoEvent auto) {
        return auto.loggedCondition("InScoringRange", inScoringRange(), true);
    }

    public Trigger inIntakeRange(AutoEvent auto) {
        return auto.loggedCondition("inIntakeRange", inIntakeRange(), true);
    }

    public Trigger flywheelsReady(AutoEvent auto) {
        return auto.loggedCondition("flywheelsReady", flywheelsReady(), true);
    }

    public Trigger hoodReady(AutoEvent auto) {
        return auto.loggedCondition("hoodReady", hoodReady(), true);
    }

    ///////////////// DRIVE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\\
    public SequentialEndingCommandGroup followChoreoPath(String pPathName, boolean pIsFirst) {
        PathPlannerPath path = getTraj(pPathName).get();
        PathPlannerTrajectory idealTraj = path.getIdealTrajectory(Drive.mRobotConfig).get();
        return new SequentialEndingCommandGroup(
            Commands.runOnce(() -> {
                if(pIsFirst) setPoseFromTrajectory(idealTraj);
            }),
            mRobotDrive.getDriveManager().followPathCommand(path)
                .withTimeout(idealTraj.getTotalTimeSeconds()),
            mRobotDrive.getDriveManager().setToStop()
        );
    }

    public SequentialEndingCommandGroup followChoreoPath(String pPathName, PPHolonomicDriveController pPID, boolean pIsFirst) {
        PathPlannerPath path = getTraj(pPathName).get();
        PathPlannerTrajectory idealTraj = path.getIdealTrajectory(Drive.mRobotConfig).get();
        return new SequentialEndingCommandGroup(
            Commands.runOnce(() -> { 
                if(pIsFirst) setPoseFromTrajectory(idealTraj);
            }),
            mRobotDrive.getDriveManager().followPathCommand(path, pPID)
                .withTimeout(idealTraj.getTotalTimeSeconds()),
            mRobotDrive.getDriveManager().setToStop()
        );
    }

    public Optional<PathPlannerPath> getTraj(String pPathName) {
        try {
            return Optional.of(PathPlannerPath.fromChoreoTrajectory(pPathName));
        } catch(Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public void setPoseFromTrajectory(PathPlannerTrajectory idealTraj) {
        mRobotDrive.setPose(AllianceFlipUtil.apply(idealTraj.sample(0.0).pose));
    }

    ///////////////// PATH CHOOSING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Supplier<Command> getAuto() {
        return getAutoChooser().get();
    }

    public LoggedDashboardChooser<Supplier<Command>> getAutoChooser() {
        return mAutoChooserLogged;
    }

    public final void tryToAddPathToChooser(String pPathName, Supplier<Command> pAuto) {
        tryToAddPathToChooser(pPathName, new Runnable() {
            @Override
            public void run() {
                mAutoChooser.addOption(pPathName, pAuto);
            }
        });
    }  
    
    /* Stops magic auton errors from occuring due to FMS or some BS I cook up */
    public void tryToAddPathToChooser(String pPathName, Runnable pPathAdding) {
        try {
            pPathAdding.run();
        } catch(Exception e) {
            mAutoChooser.addOption("Failed: "+pPathName, () -> backUpAuton());
            Telemetry.reportException(e);
        }
    }
}