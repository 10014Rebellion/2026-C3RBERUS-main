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
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.pivot.IntakePivotSS.IntakePivotStates;
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
    // private final Shooter mShooter;
    private final HoodSS mHoodSS;
    private final FuelPumpSS mFuelPumpSS;
    private final FlywheelsSS mFlywheelsSS;

    private final SendableChooser<Supplier<Command>> mAutoChooser;
    private final LoggedDashboardChooser<Supplier<Command>> mAutoChooserLogged;

    private boolean wantToShoot = false;

    public AutonCommands(Drive pRobotDrive, Intake pIntake, FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelsSS) {
        this.mRobotDrive = pRobotDrive;
        this.mIntake = pIntake;
        this.mHoodSS = pHoodSS;
        this.mFlywheelsSS = pFlywheelsSS;
        this.mFuelPumpSS = pFuelPumpSS;

        mAutoChooser = new SendableChooser<>();

        mAutoChooser.setDefaultOption("Stationary", () -> backUpAuton());
        // tryToAddPathToChooser("FirstTestPath", () -> firstPathTest("FirstPathTest", "FirstPath"));
        // tryToAddPathToChooser("FirstAuto", () -> autoTest("FirstAuto","FirstPath", "SecondPath"));
        // tryToAddPathToChooser("LeftFullPath", () -> leftFullPath());
        // tryToAddPathToChooser("RightFullPath", () -> rightFullPath());
        // tryToAddPathToChooser("Troll", () -> troll());
        // tryToAddPathToChooser("DisruptLeft", () -> disruptLeft());
        tryToAddPathToChooser("LeftDoubleSwipe", 
            () -> doubleSwipe(
                "LeftDoubleSwipe",
                "L_IT_IC_ST",
                "L_ST_IB_ST")
        );

        tryToAddPathToChooser("RightDoubleSwipe", 
            () -> doubleSwipe(
                "RightDoubleSwipe",
                "R_IT_IC_ST",
                "R_ST_IB_ST")
        );
        
        mAutoChooserLogged = new LoggedDashboardChooser<>("Autos", mAutoChooser);
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command backUpAuton() {
        return new InstantCommand();
    }

    public Command doubleSwipe(String autoName, String firstSwipe, String secondSwipe) {
        AutoEvent auto = new AutoEvent(autoName, this);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(auto.getName()+"/WantToShoot", () -> wantToShoot, true);

        SequentialEndingCommandGroup firstSwipePath = followChoreoPath(firstSwipe, true);
        Trigger isFirstSwipeRunning = auto.loggedCondition(firstSwipe+"/isRunning", () -> firstSwipePath.isRunning(), true);
        Trigger hasFirstSwipeEnded = auto.loggedCondition(firstSwipe+"/hasEnded", () -> firstSwipePath.hasEnded(), true);

        SequentialEndingCommandGroup secondSwipePath = followChoreoPath(secondSwipe, false);
        Trigger isSecondSwipeRunning = auto.loggedCondition(secondSwipe+"/isRunning", () -> secondSwipePath.isRunning(), true);
        Trigger hasSecondSwipeEnded = auto.loggedCondition(secondSwipe+"/hasEnded", () -> secondSwipePath.hasEnded(), true);

        intakingRange
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(Commands.waitSeconds(0.25).andThen(mIntake.setPivotStateCmd(IntakePivotStates.INTAKE)))
            .onFalse(mIntake.setRollerStateCmd(IntakeRollerState.IDLE));

        shootingRange
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.CLOSE_SHOT))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        autoActivted
            .onTrue(firstSwipePath)
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE))
            .onTrue(Commands.runOnce(() -> wantToShoot = false));

        hasFirstSwipeEnded
            .onTrue(Commands.runOnce(() -> wantToShoot = true))
            .onTrue(mRobotDrive.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()));

        SequentialEndingCommandGroup firstSwipeIntakeShot = 
            new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(5.0),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(0.1));

        SequentialEndingCommandGroup firstSwipeIndexShot = 
            new SequentialEndingCommandGroup(
                mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(5.0),
                mIntake.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(0.1));

        hasFirstSwipeEnded.and(() -> wantToShoot).and(() -> 
            mFlywheelsSS.atLatestClosedLoopGoal() && 
            mHoodSS.atGoal() &&
            mRobotDrive.getDriveManager().inHeadingTolerance())
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeIndexShot)
            .onTrue(mIntake.trashCompactPivotContinuous());

        hasFirstSwipeEnded.and(() -> firstSwipeIndexShot.hasEnded() && firstSwipeIntakeShot.hasEnded())
            .onTrue(Commands.runOnce(() -> wantToShoot = false))
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntake.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(secondSwipePath);

        hasSecondSwipeEnded
            .onTrue(Commands.runOnce(() -> wantToShoot = true))
            .onTrue(mRobotDrive.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()));

        SequentialEndingCommandGroup secondSwipeIntakeShot = 
            new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(5.0),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(0.1));

        SequentialEndingCommandGroup secondSwipeIndexShot = 
            new SequentialEndingCommandGroup(
                mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(5.0),
                mIntake.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(0.1));

        hasSecondSwipeEnded.and(() -> wantToShoot).and(() -> 
            mFlywheelsSS.atLatestClosedLoopGoal() && 
            mHoodSS.atGoal() &&
            mRobotDrive.getDriveManager().inHeadingTolerance())
            .onTrue(secondSwipeIntakeShot)
            .onTrue(secondSwipeIndexShot)
            .onTrue(mIntake.trashCompactPivotContinuous());

        hasSecondSwipeEnded.and(() -> secondSwipeIndexShot.hasEnded() && secondSwipeIntakeShot.hasEnded())
            .onTrue(Commands.runOnce(() -> wantToShoot = false))
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntake.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(secondSwipePath);
            
        return auto;
    }

    public Command disruptLeft() {
        AutoEvent auto = new AutoEvent("Disruptleft", this);

        Trigger autoActivated = auto.getIsRunningTrigger();

        String path1ShootingName = "ITL_IFL";
        SequentialEndingCommandGroup autoPath1 = followChoreoPath(path1ShootingName, true);
        Trigger isPath1Running = auto.loggedCondition(path1ShootingName+"/isRunning", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(path1ShootingName+"/hasEnded", () -> autoPath1.hasEnded(), true);

        autoActivated
            .onTrue(autoPath1);

        hasPath1Ended
            .onTrue(endAuto(auto));

        return auto;
    }

    public Command troll() {
        AutoEvent auto = new AutoEvent("Troll", this);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(auto.getName()+"/WantToShoot", () -> wantToShoot, true);
        Trigger scoreReady = auto.condition(shootingRange);

        String path1ShootingName = "TOWER_SHOOT";
        SequentialEndingCommandGroup autoPath1 = followChoreoPath(path1ShootingName, true);
        Trigger isPath1Running = auto.loggedCondition(path1ShootingName+"/isRunning", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(path1ShootingName+"/hasEnded", () -> autoPath1.hasEnded(), true);

        String path2Name = "OVER_BUMP";
        SequentialEndingCommandGroup autoPath2 = followChoreoPath(path2Name, false);
        Trigger isPath2Running = auto.loggedCondition(path2Name+"/isRunning", () -> autoPath2.isRunning(), true);
        Trigger hasPath2Ended = auto.loggedCondition(path2Name+"/hasEnded", () -> autoPath2.hasEnded(), true);

        String path3Name = "TROLL";
        SequentialEndingCommandGroup autoPath3 = followChoreoPath(path3Name, false);
        Trigger isPath3Running = auto.loggedCondition(path3Name+"/isRunning", () -> autoPath3.isRunning(), true);
        Trigger hasPath3Ended = auto.loggedCondition(path3Name+"/hasEnded", () -> autoPath3.hasEnded(), true);

        shootingRange
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.CLOSE_SHOT))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        autoActivted
            .onTrue(autoPath1)
            .onTrue(mIntake.setPivotStateCmd(IntakePivotStates.STOW))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE))
            .onTrue(Commands.runOnce(() -> wantToShoot = false));

        hasPath1Ended
            .onTrue(mRobotDrive.getDriveManager().setToGenericHeadingAlign(
            () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), 
            () -> GameGoalPoseChooser.getHub()))
            .onTrue(Commands.runOnce(() -> wantToShoot = true));

        SequentialEndingCommandGroup path1FPShooting = 
            new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(2.5),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(0.01));

        SequentialEndingCommandGroup path1IShooting = 
            new SequentialEndingCommandGroup(
                mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(2.5),
                mIntake.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(0.01));


        hasPath1Ended.and(shootingRange.debounce(1.0))
            .onTrue(path1FPShooting)
            .onTrue(path1IShooting);

        auto.loggedCondition(
            "Path1/ShootingHasEnded", 
                () -> path1FPShooting.hasEnded()
                && path1IShooting.hasEnded(), true)
            .onTrue(Commands.runOnce(() -> wantToShoot = false))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onTrue(autoPath2);

        hasPath2Ended
            .onTrue(autoPath3);

        hasPath3Ended
            .onTrue(endAuto(auto))
            .onTrue(mRobotDrive.getDriveManager().setToTeleop());

        return auto;
    }

    public Command rightFullPath() {
        AutoEvent auto = new AutoEvent("RightScore", this);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(auto.getName()+"/WantToShoot", () -> wantToShoot, true);
        Trigger scoreReady = auto.condition(shootingRange);

        String path1Name = "ITR_ICR";
        SequentialEndingCommandGroup autoPath1 = followChoreoPath(path1Name, true);
        Trigger isPath1Running = auto.loggedCondition(path1Name+"/isRunning", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(path1Name+"/hasEnded", () -> autoPath1.hasEnded(), true);

        String path2ShootingName = "ICR_STR";
        SequentialEndingCommandGroup autoPath2Shoot = followChoreoPath(path2ShootingName, false);
        Trigger isPath2Running = auto.loggedCondition(path2ShootingName+"/isRunning", () -> autoPath2Shoot.isRunning(), true);
        Trigger hasPath2Ended = auto.loggedCondition(path2ShootingName+"/hasEnded", () -> autoPath2Shoot.hasEnded(), true);

        autoActivted
            .onTrue(autoPath1)
            // .onTrue(Commands.waitSeconds(1.75).andThen(mIntake.setPivotStateCmd(IntakePivotState.INTAKE)))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE))
            .onTrue(Commands.runOnce(() -> wantToShoot = false));

        intakingRange
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(Commands.waitSeconds(0.25).andThen(mIntake.setPivotStateCmd(IntakePivotStates.INTAKE)))
            .onFalse(mIntake.setRollerStateCmd(IntakeRollerState.IDLE));

        shootingRange
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.CLOSE_SHOT))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        hasPath1Ended
            .onTrue(autoPath2Shoot);

        /* Shooting Logic*/
        hasPath2Ended
            .onTrue(mRobotDrive.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()))
            .onTrue(Commands.runOnce(() -> wantToShoot = true));

        SequentialEndingCommandGroup path2FPShooting = 
            new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(5.0),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(0.1));

        SequentialEndingCommandGroup path2IShooting = 
            new SequentialEndingCommandGroup(
                mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(5.0),
                mIntake.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(0.1));

        hasPath2Ended.and(shootingRange.debounce(0.5))
            .onTrue(path2FPShooting)
            .onTrue(path2IShooting)
            .onTrue(mIntake.trashCompactPivotContinuous());

        auto.loggedCondition(
            "Path2/ShootingHasEnded", 
            () -> path2FPShooting.hasEnded()
                && path2IShooting.hasEnded(), true)
            .onTrue(Commands.runOnce(() -> wantToShoot = false))
            .onTrue(mIntake.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onTrue(mRobotDrive.getDriveManager().setToTeleop())
            .onTrue(endAuto(auto));
            
        return auto;
    }

    public Command leftFullPath() {
        AutoEvent auto = new AutoEvent("LeftScore", this);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(auto.getName()+"/WantToShoot", () -> wantToShoot, true);
        Trigger scoreReady = auto.condition(shootingRange);

        String path1Name = "ITL_ICL";
        SequentialEndingCommandGroup autoPath1 = followChoreoPath(path1Name, true);
        Trigger isPath1Running = auto.loggedCondition(path1Name+"/isRunning", () -> autoPath1.isRunning(), true);
        Trigger hasPath1Ended = auto.loggedCondition(path1Name+"/hasEnded", () -> autoPath1.hasEnded(), true);

        String path2ShootingName = "ICL_STL";
        SequentialEndingCommandGroup autoPath2Shoot = followChoreoPath(path2ShootingName, false);
        Trigger isPath2Running = auto.loggedCondition(path2ShootingName+"/isRunning", () -> autoPath2Shoot.isRunning(), true);
        Trigger hasPath2Ended = auto.loggedCondition(path2ShootingName+"/hasEnded", () -> autoPath2Shoot.hasEnded(), true);

        autoActivted
            .onTrue(autoPath1)
            // .onTrue(Commands.waitSeconds(1.75).andThen(mIntake.setPivotStateCmd(IntakePivotState.INTAKE)))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VOLTAGE))
            .onTrue(Commands.runOnce(() -> wantToShoot = false));

        intakingRange
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(Commands.waitSeconds(0.25).andThen(mIntake.setPivotStateCmd(IntakePivotStates.INTAKE)))
            .onFalse(mIntake.setRollerStateCmd(IntakeRollerState.IDLE));

        shootingRange
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.CLOSE_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.CLOSE_SHOT))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        hasPath1Ended
            .onTrue(autoPath2Shoot);

        /* Shooting Logic*/
        hasPath2Ended
            .onTrue(mRobotDrive.getDriveManager().setToGenericHeadingAlign(
                () -> GameGoalPoseChooser.turnFromHub(mRobotDrive.getPoseEstimate()), 
                () -> GameGoalPoseChooser.getHub()))
            .onTrue(Commands.runOnce(() -> wantToShoot = true));

        SequentialEndingCommandGroup path2FPShooting = 
            new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(5.0),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(0.1));

        SequentialEndingCommandGroup path2IShooting = 
            new SequentialEndingCommandGroup(
                mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(5.0),
                mIntake.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(0.1));

        hasPath2Ended.and(shootingRange.debounce(0.5))
            .onTrue(path2FPShooting)
            .onTrue(path2IShooting)
            .onTrue(mIntake.trashCompactPivotContinuous());

        auto.loggedCondition(
            "Path2/ShootingHasEnded", 
            () -> path2FPShooting.hasEnded()
                && path2IShooting.hasEnded(), true)
            .onTrue(Commands.runOnce(() -> wantToShoot = false))
            .onTrue(mIntake.setPivotStateCmd(IntakePivotStates.INTAKE))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STOPPED))
            .onTrue(mRobotDrive.getDriveManager().setToTeleop())
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
        return mIntake.setPivotStateCmd(IntakePivotStates.INTAKE).alongWith(mIntake.setRollerStateCmd(IntakeRollerState.INTAKE));
    }

    public Command deployIntakeCommand() {
        return mIntake.setPivotStateCmd(IntakePivotStates.INTAKE);
    }

    public Command bindexCommand() {
        return new InstantCommand();
    }

    // public Command shotIndexCommand() {
    //     return mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).alongWith(mConveyorSS.setConveyorStateCmd(ConveyorState.INTAKE));
    // }
    
    // public Command spinFlywheelsCommand() {
    //     return mFlywheelsSS.setStateCmd(FlywheelStates.SHOOT_CLOSE);
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