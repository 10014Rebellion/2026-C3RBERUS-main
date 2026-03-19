// REBELLION 10014

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.PhoenixUtil;
import frc.robot.RobotConstants.DashboardConstants;
import frc.robot.game.TransitionTracker;
import frc.robot.systems.shooter.ShotCalculator;
import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

public class Robot extends LoggedRobot {
    private Command mAutonomousCommand;
    private RobotContainer mRobotContainer;
    private TransitionTracker mTracker;

    public Robot() {
        beginAKLogger();
        
        startWebServers();
        
        mTracker = new TransitionTracker();
        mRobotContainer = new RobotContainer();

        FollowPathCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        PhoenixUtil.refreshAll();

        ShotCalculator.getInstance().clearShootingParameters();

        ShotCalculator.getInstance().computeParameters(
            mRobotContainer.getDrivetrain().getPoseEstimate(), 
            mRobotContainer.getDrivetrain().getRobotChassisSpeeds(),
            mRobotContainer.getDrivetrain().getRobotChassisSpeedsFieldRelative()
        );

        CommandScheduler.getInstance().run();

        TransitionTracker.periodic();

        Logger.recordOutput("GameStates/TELEOPERATED TIME", mTracker.getTeleopTimeLeft());
        Logger.recordOutput("GameStates/PHASE TIME", mTracker.getTimeLeftInPhase());
        Logger.recordOutput("GameStates/AUTONOMOUS TIME", mTracker.getAutonTimeLeft());
    }

    @Override
    public void disabledInit() {
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
    }

    @Override
    public void disabledPeriodic() {
        mRobotContainer.getDrivetrain().runSwerve(Optional.of(new ChassisSpeeds()));
    }

    @Override
    public void autonomousInit() {
        mAutonomousCommand = mRobotContainer.getAutonomousCommand().get();
        TransitionTracker.autonInit();

        if (mAutonomousCommand != null) {
            CommandScheduler.getInstance().schedule(mAutonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        TransitionTracker.teleopInit();
        mRobotContainer.getDrivetrain().getDriveManager().setToTeleop();
        CommandScheduler.getInstance().schedule(mRobotContainer.getDriverProfileCommand());
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}

    private void beginAKLogger () {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (RobotConstants.kCurrentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                SignalLogger.stop();
                break;

            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();
    }

    private void startWebServers() {
        if (DashboardConstants.kDashboardEnabled) {
            System.out.println("Starting Dashboard Webserver");
            System.out.println("Dashboard directory: "
                    + Filesystem.getDeployDirectory()
                    + "/"
                    + DashboardConstants.kDashboardPath);
            try {
                WebServer.start(
                        DashboardConstants.kDashboardPort,
                        Filesystem.getDeployDirectory() + "/" + DashboardConstants.kDashboardPath);
                if (Robot.isSimulation()) {
                    java.awt.Desktop.getDesktop()
                            .browse(java.net.URI.create("http://127.0.0.1:" + DashboardConstants.kDashboardPort));
                }
            } catch (Exception e) {
                System.out.println("Dashboard Webserver failed to start: " + e.getMessage());
            }
        }

        if (DashboardConstants.kDeployServerEnabled) {
            System.out.println("Starting Deploy Webserver");
            System.out.println("Deploy directory: "
                    + Filesystem.getDeployDirectory()
                    + "/"
                    + DashboardConstants.kDeployServerPath);
            try {
                WebServer.start(
                        DashboardConstants.kDeployServerPort,
                        Filesystem.getDeployDirectory() + "/" + DashboardConstants.kDeployServerPath);
                if (Robot.isSimulation()) {
                    java.awt.Desktop.getDesktop()
                            .browse(java.net.URI.create("http://127.0.0.1:" + DashboardConstants.kDeployServerPort));
                }
            } catch (Exception e) {
                System.out.println("Deploy Webserver failed to start: " + e.getMessage());
            }
        }
    }
}
