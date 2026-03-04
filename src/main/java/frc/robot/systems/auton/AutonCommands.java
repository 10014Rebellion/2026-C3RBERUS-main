package frc.robot.systems.auton;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.lib.telemetry.Telemetry;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoEvent;
import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.lib.math.AllianceFlipUtil;

public class AutonCommands extends SubsystemBase {
    private final Drive mRobotDrive;
    private final Intake mIntake;
    // private final Shooter mShooter;
    private final ConveyorSS mConveyorSS;

    private final SendableChooser<Supplier<Command>> mAutoChooser;
    private final LoggedDashboardChooser<Supplier<Command>> mAutoChooserLogged;

    public AutonCommands(Drive pRobotDrive, Intake pIntake, ConveyorSS mConveyor) {
        this.mRobotDrive = pRobotDrive;
        this.mIntake = pIntake;
        // this.mShooter = pShooter;
        this.mConveyorSS = mConveyor;

        mAutoChooser = new SendableChooser<>();

        mAutoChooser.setDefaultOption("Stationary", () -> backUpAuton());
        tryToAddPathToChooser("FirstTestPath", () -> firstPathTest("FirstPathTest", "FirstPath"));
        tryToAddPathToChooser("FirstAuto", () -> autoTest("FirstAuto","FirstPath", "SecondPath"));
        tryToAddPathToChooser("LeftFullPath", () -> leftFullPath());
        
        mAutoChooserLogged = new LoggedDashboardChooser<>("Autos", mAutoChooser);
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command backUpAuton() {
        return new InstantCommand();
    }

    public Command leftFullPath() {
        AutoEvent auto = new AutoEvent("LeftScore", this);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = inIntakeRange(auto);
        Trigger shootingRange = inScoringRange(auto);

        String path1Name = "ITL_IFL";
        SequentialEndingCommandGroup autoPath1 = followChoreoPath(path1Name, true);
        Trigger isPath1Running = auto.loggedCondition(path1Name+"/isRunning", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(path1Name+"/hasEnded", () -> autoPath1.hasEnded(), true);

        String path2Name = "IFL_STL";
        SequentialEndingCommandGroup autoPath2Shoot = followChoreoPath(path2Name, false);
        Trigger isPath2Running = auto.loggedCondition(path2Name+"/isRunning", () -> autoPath2Shoot.isRunning(), true);
        Trigger hasPath2Ended = auto.loggedCondition(path2Name+"/hasEnded", () -> autoPath2Shoot.hasEnded(), true);

        String path3Name = "STL_ICL";
        SequentialEndingCommandGroup autoPath3 = followChoreoPath(path3Name, false);
        Trigger isPath3Running = auto.loggedCondition(path3Name+"/isRunning", () -> autoPath3.isRunning(), true);
        Trigger hasPath3Ended = auto.loggedCondition(path3Name+"/hasEnded", () -> autoPath3.hasEnded(), true);

        String path4Name = "ICL_STL";
        SequentialEndingCommandGroup autoPath4Shoot = followChoreoPath(path4Name, false);
        Trigger isPath4RRunning = auto.loggedCondition(path4Name+"/isRunning", () -> autoPath4Shoot.isRunning(), true);
        Trigger hasPath4Ended = auto.loggedCondition(path4Name+"/hasEnded", () -> autoPath4Shoot.hasEnded(), true);

        String path5Name = "STL_IBL";
        SequentialEndingCommandGroup autoPath5 = followChoreoPath(path5Name, false);
        Trigger isPath5Running = auto.loggedCondition(path5Name+"/isRunning", () -> autoPath5.isRunning(), true);
        Trigger hasPath5Ended = auto.loggedCondition(path5Name+"/hasEnded", () -> autoPath5.hasEnded(), true);

        String path6Name = "IBL_STL";
        SequentialEndingCommandGroup autoPath6Shoot = followChoreoPath(path6Name, false);
        Trigger isPath6Running = auto.loggedCondition(path6Name+"/isRunning", () -> autoPath6Shoot.isRunning(), true);
        Trigger hasPath6Ended = auto.loggedCondition(path6Name+"/hasEnded", () -> autoPath6Shoot.hasEnded(), true);

        // autoActivted
        //     .onTrue(autoPath1)
        //     .onTrue(mIntake.setPivotStateCmd(IntakePivotState.INTAKE))
        //     .onTrue(mShooter.setFlywheelStateCmd(FlywheelState.STANDBY));

        // intakingRange
        //     .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.INTAKE))
        //     .onTrue(mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE))
        //     .onFalse(mIntake.setRollerStateCmd(IntakeRollerState.IDLE))
        //     .onFalse(mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE));

        // shootingRange
        //     .onTrue(mShooter.setFlywheelStateCmd(FlywheelState.SHOOT_FAR))
        //     .onTrue(mShooter.setHoodStateCmd(HoodState.MAX))
        //     .onFalse(mShooter.setFlywheelStateCmd(FlywheelState.STANDBY))
        //     .onFalse(mShooter.setHoodStateCmd(HoodState.MIN));

        // hasPath1Ended
        //     .onTrue(autoPath2Shoot);

        // /* Shooting Logic*/
        // hasPath2Ended
        //     .onTrue(mRobotDrive.getDriveManager().setToGenericHeadingAlign(
        //         () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), 
        //         () -> GameGoalPoseChooser.getHub()));

        // SequentialEndingCommandGroup path2FPShooting = 
        //     new SequentialEndingCommandGroup(
        //         mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE).withTimeout(1.0),
        //         mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE));

        // SequentialEndingCommandGroup path2IShooting = 
        //     new SequentialEndingCommandGroup(
        //         mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(1.0),
        //         mIntake.setRollerStateCmd(IntakeRollerState.IDLE));

        // SequentialEndingCommandGroup path2CShooting = 
        //     new SequentialEndingCommandGroup(
        //         mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE).withTimeout(1.0),
        //         mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE));

        // hasPath2Ended.and(
        //     new Trigger(
        //         () -> mShooter.getIsHoodAtGoal() && 
        //         mShooter.getIsFlywheelAtGoal() && 
        //         mRobotDrive.getDriveManager().inHeadingTolerance()))
        //     .onTrue(path2FPShooting)
        //     .onTrue(path2IShooting)
        //     .onTrue(path2CShooting)
        //     .onTrue(mIntake.setPivotStateCmd(IntakePivotState.COMPACT));

        // auto.loggedCondition(
        //     "Path2/ShootingHasEnded", 
        //     () -> path2CShooting.hasEnded() 
        //         && path2FPShooting.hasEnded()
        //         && path2IShooting.hasEnded(), true)
        //     .onTrue(mIntake.setPivotStateCmd(IntakePivotState.INTAKE))
        //     .onTrue(autoPath3);

        // hasPath3Ended
        //     .onTrue(autoPath4Shoot);

        // hasPath4Ended
        //     .onTrue(mRobotDrive.getDriveManager().setToGenericHeadingAlign(
        //         () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), 
        //         () -> GameGoalPoseChooser.getHub()));

        // SequentialEndingCommandGroup path4FPShooting = 
        //     new SequentialEndingCommandGroup(
        //         mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE).withTimeout(1.0),
        //         mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE));

        // SequentialEndingCommandGroup path4IShooting = 
        //     new SequentialEndingCommandGroup(
        //         mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(1.0),
        //         mIntake.setRollerStateCmd(IntakeRollerState.IDLE));

        // SequentialEndingCommandGroup path4CShooting = 
        //     new SequentialEndingCommandGroup(
        //         mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE).withTimeout(1.0),
        //         mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE));

        // hasPath4Ended.and(
        //     new Trigger(
        //         () -> mShooter.getIsHoodAtGoal() && 
        //         mShooter.getIsFlywheelAtGoal() && 
        //         mRobotDrive.getDriveManager().inHeadingTolerance()))
        //     .onTrue(path4FPShooting)
        //     .onTrue(path4IShooting)
        //     .onTrue(path4CShooting)
        //     .onTrue(mIntake.setPivotStateCmd(IntakePivotState.COMPACT));

        // auto.loggedCondition(
        //     "Path4/ShootingHasEnded", 
        //     () -> path4CShooting.hasEnded() 
        //         && path4FPShooting.hasEnded()
        //         && path4IShooting.hasEnded(), true)
        //     .onTrue(mIntake.setPivotStateCmd(IntakePivotState.INTAKE))
        //     .onTrue(autoPath5);

        // hasPath5Ended
        //     .onTrue(autoPath6Shoot);

        // hasPath6Ended
        //     .onTrue(mRobotDrive.getDriveManager().setToGenericHeadingAlign(
        //         () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), 
        //         () -> GameGoalPoseChooser.getHub()));

        // SequentialEndingCommandGroup path6FPShooting = 
        //     new SequentialEndingCommandGroup(
        //         mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE).withTimeout(1.0),
        //         mShooter.setFuelPumpStateCmd(FuelPumpState.IDLE));

        // SequentialEndingCommandGroup path6IShooting = 
        //     new SequentialEndingCommandGroup(
        //         mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(1.0),
        //         mIntake.setRollerStateCmd(IntakeRollerState.IDLE));

        // SequentialEndingCommandGroup path6CShooting = 
        //     new SequentialEndingCommandGroup(
        //         mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE).withTimeout(1.0),
        //         mConveyorSS.setConveyorStateCmd(ConveyorState.IDLE));

        // hasPath6Ended.and(
        //     new Trigger(
        //         () -> mShooter.getIsHoodAtGoal() && 
        //         mShooter.getIsFlywheelAtGoal() && 
        //         mRobotDrive.getDriveManager().inHeadingTolerance()))
        //     .onTrue(path6FPShooting)
        //     .onTrue(path6IShooting)
        //     .onTrue(path6CShooting)
        //     .onTrue(mIntake.setPivotStateCmd(IntakePivotState.COMPACT));

        // auto.loggedCondition(
        //     "Path6/ShootingHasEnded", 
        //     () -> path6CShooting.hasEnded() 
        //         && path6FPShooting.hasEnded()
        //         && path6IShooting.hasEnded(), true)
        //     .onTrue(mIntake.setPivotStateCmd(IntakePivotState.INTAKE))
        //     .onTrue(endAuto(auto));
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

    // public Command shotIndexCommand() {
    //     return mShooter.setFuelPumpStateCmd(FuelPumpState.INTAKE).alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE));
    // }
    
    // public Command spinFlywheelsCommand() {
    //     return mShooter.setFlywheelStateCmd(FlywheelState.SHOOT_CLOSE);
    // }

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

    // public Trigger inIntakeRange(AutoEvent auto) {
    //     return auto.loggedCondition("inIntakeRange", inIntakeRange(), true);
    // }

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