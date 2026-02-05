package frc.robot.auton;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.lib.telemetry.Telemetry;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoEvent;
import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.systems.drive.Drive;
import frc.lib.math.AllianceFlipUtil;

public class AutonCommands extends SubsystemBase {
    private final Drive mRobotDrive;

    private final SendableChooser<Supplier<Command>> mAutoChooser;
    private final LoggedDashboardChooser<Supplier<Command>> mAutoChooserLogged;

    public AutonCommands(Drive pRobotDrive) {
        this.mRobotDrive = pRobotDrive;

        mAutoChooser = new SendableChooser<>();

        mAutoChooser.setDefaultOption("Stationary", () -> backUpAuton());
        tryToAddPathToChooser("FirstTestPath", () -> firstPathTest("FirstPathTest", "FirstPath", Rotation2d.kZero));
        tryToAddPathToChooser("FirstAuto", () -> autoTest("FirstAuto","FirstPath", Rotation2d.kZero, "SecondPath"));
        
        mAutoChooserLogged = new LoggedDashboardChooser<>("Autos", mAutoChooser);
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command backUpAuton() {
        return new InstantCommand();
    }

    public Command firstPathTest(String pAutoName, String pName, Rotation2d pInitRot) {
        AutoEvent auto = new AutoEvent(pAutoName, this);
        SequentialEndingCommandGroup autoPath1 = followFirstChoreoPath(pName, pInitRot);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger isPath1Running = auto.loggedCondition(pName+"IsFinished", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(pName+"IsFinished", () -> autoPath1.hasEnded(), true);

        autoActivted
            .onTrue(autoPath1);

        hasPath1Ended
            .onTrue(Commands.runOnce(() -> auto.cancel()));

        return auto;
    }

    public Command autoTest(String pAutoName, String pName1, Rotation2d pInitRot, String pName2) {
        AutoEvent auto = new AutoEvent(pAutoName, this);
        SequentialEndingCommandGroup autoPath1 = followFirstChoreoPath(pName1, pInitRot);
        SequentialEndingCommandGroup autoPath2 = followChoreoPath(pName2);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger isPath1Running = auto.loggedCondition(pName1+"IsFinished", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(pName1+"IsFinished", () -> autoPath1.hasEnded(), true);

        Trigger isPath2Running = auto.loggedCondition(pName2+"IsFinished", () -> autoPath2.isRunning(), true);
        Trigger hasPath2Ended = auto.loggedCondition(pName2+"IsFinished", () -> autoPath2.hasEnded(), true);

        autoActivted
            .onTrue(autoPath1);
        
        hasPath1Ended
            .onTrue(autoPath2);

        hasPath2Ended
            .onTrue(Commands.runOnce(() -> auto.cancel()));

        return auto;
    }

    ///////////////// SUPERSTRUCTURE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\
    public Command intakeCommand() {
        return new InstantCommand();
    }

    public Command bindexCommand() {
        return new InstantCommand();
    }

    public Command shotIndexCommand() {
        return new InstantCommand();
    }
    
    public Command shootCommand() {
        return new InstantCommand();
    }

    public Command turnToHubCommand() {
        return new InstantCommand();
    }

    ///////////////// DRIVE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\\
    public SequentialEndingCommandGroup followFirstChoreoPath(String pPathName, Rotation2d startingRotation) {
        PathPlannerPath path = getTraj(pPathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();

        return new SequentialEndingCommandGroup(
            new InstantCommand(() -> {
                mRobotDrive.setPose(AllianceFlipUtil.apply(
                    new Pose2d(
                        path.getPathPoses().get(0).getTranslation(), 
                        startingRotation)));
            }), 
            mRobotDrive.customFollowPathCommand(path).withTimeout(totalTimeSeconds), 
            mRobotDrive.setToStop());
    }

    public SequentialEndingCommandGroup followChoreoPath(String pPathName) {
        PathPlannerPath path = getTraj(pPathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();
        return new SequentialEndingCommandGroup(
            mRobotDrive.customFollowPathCommand(path).withTimeout(totalTimeSeconds),
            mRobotDrive.setToStop()
        );
    }

    public SequentialEndingCommandGroup followChoreoPath(String pPathName, PPHolonomicDriveController pPID) {
        PathPlannerPath path = getTraj(pPathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();
        return new SequentialEndingCommandGroup(
            mRobotDrive.customFollowPathCommand(path, pPID).withTimeout(totalTimeSeconds),
            mRobotDrive.setToStop()
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