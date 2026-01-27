package frc.robot.auton;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoEvent;
import frc.robot.systems.drive.Drive;
import frc.lib.math.AllianceFlipUtil;

public class AutonCommands extends SubsystemBase {
    public static enum AutoState {
        PreScoreCoral,
        ScoreCoral,
        PreIntakeCoral,
        IntakeCoral
    }

    private final Drive mRobotDrive;

    private final SendableChooser<Command> mAutoChooser;
    private final LoggedDashboardChooser<Command> mAutoChooserLogged;

    public AutonCommands(Drive pRobotDrive) {
        this.mRobotDrive = pRobotDrive;

        mAutoChooser = new SendableChooser<>();

        mAutoChooser.setDefaultOption("Stationary", backUpAuton());
        tryToAddPathToChooser("NewPath", firstPathTest("NewPath", Rotation2d.kZero));
        
        mAutoChooserLogged = new LoggedDashboardChooser<>("Autos", mAutoChooser);
    }

    public Command getAuto() {
        return mAutoChooserLogged.get();
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
    public Command backUpAuton() {
        return new InstantCommand();
    }

    public Command firstPathTest(String pName, Rotation2d rot) {
        AutoEvent auto = new AutoEvent(this);
        Command autoPath = followFirstChoreoPath(pName, rot);

        auto.getIsRunningTrigger()
            .onTrue(autoPath);
        
        auto.condition(() -> autoPath.isFinished())
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

    ///////////////// PATH CREATION LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command followFirstChoreoPath(String pathName, Rotation2d startingRotation) {
        PathPlannerPath path = getTraj(pathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                mRobotDrive.setPose(AllianceFlipUtil.apply(new Pose2d(path.getPathPoses().get(0).getTranslation(), startingRotation)));
            }), 
            mRobotDrive.customFollowPathCommand(path).withTimeout(totalTimeSeconds), 
            mRobotDrive.setToStop());
    }

    public Command followChoreoPath(String pathName) {
        PathPlannerPath path = getTraj(pathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();
        return 
            mRobotDrive.customFollowPathCommand(path).withTimeout(totalTimeSeconds).andThen(
            mRobotDrive.setToStop());
    }

    public Command followChoreoPath(String pathName, PPHolonomicDriveController PID) {
        PathPlannerPath path = getTraj(pathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.mRobotConfig).get().getTotalTimeSeconds();
        return 
            mRobotDrive.customFollowPathCommand(path, PID).withTimeout(totalTimeSeconds).andThen(
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
}