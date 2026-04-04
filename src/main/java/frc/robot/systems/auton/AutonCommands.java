package frc.robot.systems.auton;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.lib.telemetry.Telemetry;

import frc.robot.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoEvent;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.lib.math.AllianceFlipUtil;

public class AutonCommands extends SubsystemBase {
    private final Drive mRobotDrive;
    private final Intake mIntake;
    private final HoodSS mHoodSS;
    private final FuelPumpSS mFuelPumpSS;
    private final FlywheelsSS mFlywheelsSS;

    private final SendableChooser<Supplier<Command>> mAutoChooser;
    private final LoggedDashboardChooser<Supplier<Command>> mAutoChooserLogged;

    private final AutoFactory mAutoFactory;

    private String[] usedPathNames = new String[] {
        "L_IT_IC_ST",
        "L_ST_IB_ST",
        "L_ST_IB_ST_BUMP",
        "R_IT_IC_ST",
        "R_ST_IB_ST",
        "R_ST_IB_ST_BUMP"
    };

    public AutonCommands(Drive pRobotDrive, Intake pIntake, FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelsSS) {
        this.mRobotDrive = pRobotDrive;
        this.mIntake = pIntake;
        this.mHoodSS = pHoodSS;
        this.mFlywheelsSS = pFlywheelsSS;
        this.mFuelPumpSS = pFuelPumpSS;

        mAutoFactory = new AutoFactory(
            mRobotDrive::getPoseEstimate, 
            mRobotDrive::setPose, 
            (SwerveSample s) -> {
                Logger.recordOutput(
                    "Drive/Choreo/TrajectorySetpoint", 
                    new Pose2d(
                        s.x, s.y, Rotation2d.fromRadians(s.heading)
                    ));                
                mRobotDrive.getDriveManager().setToAuton();
                mRobotDrive.getDriveManager().setPPDesiredSpeeds(
                    mRobotDrive
                        .getDriveManager()
                        .getChoreoHolonomicController()
                        .calculateFromSwerveSample(
                            s, mRobotDrive.getPoseEstimate()));
                mRobotDrive.setDriveFeedforwardsFromChoreo(s);
            }, 
            true, 
            mRobotDrive,
            (sample, isStart) -> {
                Logger.recordOutput("Drive/Choreo/Trajectory", sample.getPoses());
                Logger.recordOutput("Drive/Choreo/IsTrajectoryRunning", isStart);
            });

        loadCacheForAllPaths();

        mAutoChooser = new SendableChooser<>();

        mAutoChooser.setDefaultOption("Stationary", () -> backUpAuton());

        SingleSwipe mLeftSingleSwipeAuto = 
            new SingleSwipe(
                this, 
                "LeftSingleSwipe", 
                "L_IT_IC_ST", 
                4.2);

        SingleSwipe mRightSingleSwipeAuto = 
            new SingleSwipe(
                this, 
                "RightSingleSwipe", 
                "R_IT_IC_ST", 
                4.12);

        DoubleSwipe mLeftDoubleSwipeAuto =
            new DoubleSwipe(
                this, 
                "LeftDoubleSwipeBump", 
                "L_IT_IC_ST", 
                4.2,
                "L_ST_IB_ST_BUMP",
                4.59);

        DoubleSwipe mRightDoubleSwipeAuto =
            new DoubleSwipe(
                this, 
                "RightDoubleSwipeBump", 
                "R_IT_IC_ST", 
                4.12,
                "R_ST_IB_ST_BUMP",
                4.33);

        tryToAddPathToChooser("LeftSingleSwipe", 
            () -> mLeftSingleSwipeAuto.getAuton()
        );

        tryToAddPathToChooser("LeftDoubleSwipeBump", 
            () -> mLeftDoubleSwipeAuto.getAuton()
        );

        tryToAddPathToChooser("RightSingleSwipe", 
            () -> mRightSingleSwipeAuto.getAuton()
        );

        tryToAddPathToChooser("RightDoubleSwipeBump", 
            () -> mRightDoubleSwipeAuto.getAuton()
        );
        
        mAutoChooserLogged = new LoggedDashboardChooser<>("Autos", mAutoChooser);
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command backUpAuton() {
        return new InstantCommand();
    }

    ///////////////// SUPERSTRUCTURE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\
    public Command endAuto(AutoEvent auto) {
        return Commands.runOnce(() -> auto.cancel());
    }

    public Trigger inIntakeRange(AutoEvent auto) {
        return auto.loggedCondition(auto.getName()+"/InIntakeRange", 
        () -> {
            if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
                return mRobotDrive.getPoseEstimate().getX() > 5.0;
            } else {
                return mRobotDrive.getPoseEstimate().getX() < AllianceFlipUtil.applyX(5.0);
            }
        }, 
        true);
    }

    public Trigger inScoringRange(AutoEvent auto) {
        return auto.loggedCondition(
            "InScoringRange", 
            () -> {
                if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
                    return mRobotDrive.getPoseEstimate().getX() < 4.0;
                } else {
                    return mRobotDrive.getPoseEstimate().getX() > AllianceFlipUtil.applyX(4.0);
                }
            }, 
            true);
    }

    ///////////////// DRIVE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\\
    public FollowPathCommand followChoreoPath(
            String pPathName, boolean pIsFirst, AutoEvent pAuto) {
        return mRobotDrive.getDriveManager().followPathCommand(
            getTraj(pPathName).get(), 
            pIsFirst,
            pAuto);
    }

    public FollowPathCommand followChoreoPath(
            String pPathName, PPHolonomicDriveController pPID, boolean pIsFirst, AutoEvent pAuto) {
        return mRobotDrive.getDriveManager().followPathCommand(
            getTraj(pPathName).get(),
            pPID, 
            pIsFirst,
            pAuto);
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

    /* Subsystem Getters */
    public Drive getDriveSubsystem() {
        return mRobotDrive;
    }

    public FlywheelsSS getFlywheelSubsystem() {
        return mFlywheelsSS;
    }

    public HoodSS getHoodSubsystem() {
        return mHoodSS;
    }

    public FuelPumpSS getFuelPumpSubsystem() {
        return mFuelPumpSS;
    }

    public Intake getIntakeSubsystem() {
        return mIntake;
    }

    public AutoFactory getAutoFactory() {
        return mAutoFactory;
    }

    private void loadCacheForAllPaths() {
        for(int i = 0; i < usedPathNames.length; i++) {
            mAutoFactory.trajectoryCmd(usedPathNames[i]);
            getTraj(usedPathNames[i]);
        }
    }
}