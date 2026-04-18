package frc.robot.systems.auton;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.lib.telemetry.Telemetry;

import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.game.FieldConstants;
import frc.robot.game.GameGoalPoseChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoEvent;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.efi.FuelInjectorSS;
import frc.robot.systems.efi.FuelInjectorSS.FuelInjectorState;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;
import frc.lib.math.AllianceFlipUtil;

public class AutonCommands extends SubsystemBase {
    private final Drive mRobotDrive;
    private final Intake mIntake;
    private final HoodSS mHoodSS;
    private final FuelPumpSS mFuelPumpSS;
    private final FlywheelsSS mFlywheelsSS;
    private final ClimbSS mClimbSS;
    private final FuelInjectorSS mFuelInjectorSS;

    private final SendableChooser<Supplier<Command>> mAutoChooser;
    private final LoggedDashboardChooser<Supplier<Command>> mAutoChooserLogged;

    private final AutoFactory mAutoFactory;

    private String[] usedPathNames = new String[] {
        "L_IT_IC_ST",
        "L_ST_IB_ST_Snake",
        "L_ST_IB_ST_Snake2",
        "L_ST_IB_ST",
        "L_ST_IB_ST_BUMP",
        "R_IT_IC_ST",
        "R_ST_IB_ST",
        "R_ST_IB_ST_BUMP"
    };

    public AutonCommands(Drive pRobotDrive, Intake pIntake, FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelsSS, ClimbSS pClimbSS, FuelInjectorSS pInjectorSS) {
        this.mRobotDrive = pRobotDrive;
        this.mIntake = pIntake;
        this.mHoodSS = pHoodSS;
        this.mFlywheelsSS = pFlywheelsSS;
        this.mFuelPumpSS = pFuelPumpSS;
        this.mClimbSS = pClimbSS;
        this.mFuelInjectorSS = pInjectorSS;

        mAutoFactory = new AutoFactory(
            mRobotDrive::getPoseEstimate, 
            mRobotDrive::setPose, 
            (SwerveSample s) -> {}, 
            true, 
            mRobotDrive,
            (sample, isStart) -> {
                Logger.recordOutput("Drive/Choreo/Trajectory", sample.getPoses());
                Logger.recordOutput("Drive/Choreo/IsTrajectoryRunning", isStart);
            });

        loadCacheForAllPaths();

        mAutoChooser = new SendableChooser<>();
        
        Command mDeployAndIntake = new ParallelCommandGroup(
            mIntake.setRackStateCmd(IntakeRackState.INTAKE, false),
            mIntake.setRollerStateCmd(IntakeRollerState.INTAKE, false)
            ).andThen(new WaitCommand(0.2));
            
        Command mGoToRecovery = mRobotDrive.getDriveManager().setToGenericAutoAlign(
            () -> new Pose2d(6.586979389190674, 5.464938640594482, Rotation2d.kZero),
            ConstraintType.LINEAR);

        Command mRevUp = new ParallelCommandGroup(
            mFlywheelsSS.setStateCmd(FlywheelStates.BUMP_VELOCITY, false),
            mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT, false)
        );
                
        Command mAlignandShoot = new ParallelCommandGroup(
            mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY, false),
            mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION, false),
            mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT, false),

            mRobotDrive.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()),
                () -> GameGoalPoseChooser.getHub()
            ).until(mRobotDrive.getDriveManager().waitUntilHeadingAlignFinishes()).andThen(
                mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE).alongWith(
                    mIntake.anshulCompact()
                )
            )
        );      
                
        NamedCommands.registerCommand("DeployAndIntake", mDeployAndIntake);            
        NamedCommands.registerCommand("GoToRecovery", mGoToRecovery);
        NamedCommands.registerCommand("RevUp", mRevUp);
        NamedCommands.registerCommand("AlignAndShoot", mAlignandShoot);

        mAutoChooser.addOption("TL-SINGLE-BUMP", () -> AutoBuilder.buildAuto("TL-SINGLE-BUMP"));

        SingleSwipe mLeftSingleSwipeAuto = 
            new SingleSwipe(
                this, 
                "LeftSingleSwipe", 
                "L_IT_IC_ST", 
                4.94,
                () -> GameGoalPoseChooser.leftTrenchApproachPose(),
                () -> GameGoalPoseChooser.leftTrenchExitPose());

        SingleSwipeClimb mLeftSingleSwipeClimbAuto =
            new SingleSwipeClimb(
                this, 
                "LeftSingleSwipeClimb", 
                "L_IT_IC_ST", 
                4.94, 
                () -> GameGoalPoseChooser.leftTrenchApproachPose(),
                () -> GameGoalPoseChooser.leftTrenchExitPose(),
                FieldConstants.kClimbLeftPose);
        
        // LEFT
        double kLeftSecondShootTimestamp = 3.9; // L_ST_IB_ST_BUMP
        DoubleSwipe mLeftDoubleSwipeBumpAuto =
            new DoubleSwipe(
                this, 
                "LeftDoubleSwipeBump", 
                "L_IT_IC_ST", 
                4.94,
                () -> GameGoalPoseChooser.leftTrenchApproachPose(),
                () -> GameGoalPoseChooser.leftTrenchExitPose(),
                "L_ST_IB_ST_BUMP",
                kLeftSecondShootTimestamp);
        DoubleSwipeClimb mLeftDoubleSwipeBumpClimbAuto =
            new DoubleSwipeClimb(
                this, 
                "LeftDoubleSwipeBumpClimb", 
                "L_IT_IC_ST", 
                4.93,
                () -> GameGoalPoseChooser.leftTrenchApproachPose(),
                () -> GameGoalPoseChooser.leftTrenchExitPose(),
                "L_ST_IB_ST_BUMP",
                kLeftSecondShootTimestamp,
                FieldConstants.kClimbLeftPose);

        // RIGHT
        SingleSwipe mRightSingleSwipeAuto = 
            new SingleSwipe(
                this, 
                "RightSingleSwipe", 
                "R_IT_IC_ST", 
                4.83, 
                () -> GameGoalPoseChooser.rightTrenchApproachPose(),
                () -> GameGoalPoseChooser.rightTrenchExitPose());

        SingleSwipeClimb mRightSingleSwipeClimbAuto =
            new SingleSwipeClimb(
                this, 
                "RightSingleSwipeClimb", 
                "R_IT_IC_ST", 
                4.83, 
                () -> GameGoalPoseChooser.rightTrenchApproachPose(),
                () -> GameGoalPoseChooser.rightTrenchExitPose(),
                FieldConstants.kClimbRightPose);

        DoubleSwipe mRightDoubleSwipeBumpAuto =
            new DoubleSwipe(
                this, 
                "RightDoubleSwipeBump", 
                "R_IT_IC_ST", 
                4.83,
                () -> GameGoalPoseChooser.rightTrenchApproachPose(),
                () -> GameGoalPoseChooser.rightTrenchExitPose(),
                "R_ST_IB_ST_BUMP",
                4.8);

        DoubleSwipeClimb mRightDoubleSwipeBumpClimbAuto =
            new DoubleSwipeClimb(
                this, 
                "RightDoubleSwipeBumpClimb", 
                "R_IT_IC_ST", 
                4.83,
                () -> GameGoalPoseChooser.rightTrenchApproachPose(),
                () -> GameGoalPoseChooser.rightTrenchExitPose(),
                "R_ST_IB_ST_BUMP",
                4.8,
                FieldConstants.kClimbRightPose);

        SnakeSwipe mLeftSnakeSwipe = new SnakeSwipe(
            this, 
            "LeftSnakeSwipe", 
            "L_IT_IC_ST_Snake", 
            4.1);

        DoubleSnakeSwipe mLeftDoubleSnakeSwipe = new DoubleSnakeSwipe(
            this, 
            "LeftDoubleSnakeSwipe", 
            "L_IT_IC_ST_Snake", 
            4.1,
            "L_IT_IC_ST_Snake2", 
            3.2);

        SnakeSwipe mRightSnakeSwipe = new SnakeSwipe(
            this, 
            "RightSnakeSwipe", 
            "R_IT_IC_ST_Snake", 
            6.6);
    
        DoubleSnakeSwipe mRightDoubleSnakeSwipe = new DoubleSnakeSwipe(
            this, 
            "RightDoubleSnakeSwipe", 
            "R_IT_IC_ST_Snake", 
            6.6,
            "R_IT_IC_ST_Snake2",                 
            3.2);

        tryToAddPathToChooser(
            "LeftDoubleSnakeSwipe", 
            () -> mLeftDoubleSnakeSwipe.getAuton()
        );

        tryToAddPathToChooser(
            "LeftSnakeSwipe", 
            () -> mLeftSnakeSwipe.getAuton()
        );

        tryToAddPathToChooser(
            "RightDoubleSnakeSwipe", 
            () -> mRightDoubleSnakeSwipe.getAuton()
        );

        tryToAddPathToChooser(
            "RightSnakeSwipe", 
            () -> mRightSnakeSwipe.getAuton()
        );

        tryToAddPathToChooser("LeftSingleSwipe", 
            () -> mLeftSingleSwipeAuto.getAuton()
        );

        tryToAddPathToChooser("LeftSingleSwipeClimb", 
            () -> mLeftSingleSwipeClimbAuto.getAuton()
        );

        tryToAddPathToChooser("LeftDoubleSwipeBump", 
            () -> mLeftDoubleSwipeBumpAuto.getAuton()
        );

        tryToAddPathToChooser("LeftDoubleSwipeBumpClimb", 
            () -> mLeftDoubleSwipeBumpClimbAuto.getAuton()
        );

        tryToAddPathToChooser("RightSingleSwipe", 
            () -> mRightSingleSwipeAuto.getAuton()
        );

        tryToAddPathToChooser("RightSingleSwipeClimb", 
            () -> mRightSingleSwipeClimbAuto.getAuton()
        );

        tryToAddPathToChooser("RightDoubleSwipeBump", 
            () -> mRightDoubleSwipeBumpAuto.getAuton()
        );

        tryToAddPathToChooser("RightDoubleSwipeBumpClimb", 
            () -> mRightDoubleSwipeBumpClimbAuto.getAuton()
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

    /* Shoot Commands */
    public SequentialEndingCommandGroup timedIndexShot(double timeout, double endTimeout) {
        return new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(timeout),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(endTimeout));
    }

    public SequentialEndingCommandGroup timedIntakeShot(double timeout, double endTimeout) {
        return new SequentialEndingCommandGroup(
            mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(timeout),
            mIntake.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(endTimeout));
    }

    public SequentialEndingCommandGroup timedInjectorShot(double timeout, double endTimeout) {
        return new SequentialEndingCommandGroup(
            mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE).withTimeout(timeout),
            mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE).withTimeout(endTimeout));
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

    public ClimbSS getClimbSubsystem() {
        return mClimbSS;
    }

    public FuelInjectorSS getFuelInjector() {
        return mFuelInjectorSS;
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