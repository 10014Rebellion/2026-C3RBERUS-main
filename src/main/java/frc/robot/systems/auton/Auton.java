package frc.robot.systems.auton;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.commands.AutoEvent;
import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.hood.HoodSS;

public class Auton {
    protected final AutonCommands mAutos;
    protected final Drive mDriveSS;
    protected final FlywheelsSS mFlywheelsSS;
    protected final HoodSS mHoodSS;
    protected final FuelPumpSS mFuelPumpSS;
    protected final Intake mIntakeSS;

    public Auton(AutonCommands pAutos) {
        mAutos = pAutos;

        this.mDriveSS = pAutos.getDriveSubsystem();
        this.mFlywheelsSS = pAutos.getFlywheelSubsystem();
        this.mHoodSS = pAutos.getHoodSubsystem();
        this.mFuelPumpSS = pAutos.getFuelPumpSubsystem();
        this.mIntakeSS = pAutos.getIntakeSubsystem();
    }

    protected SequentialEndingCommandGroup followChoreoPath(String pPathName, boolean pIsFirst) {
        return mAutos.followChoreoPath(pPathName, pIsFirst);
    }

    protected SequentialEndingCommandGroup followChoreoPath(String pPathName, PPHolonomicDriveController pPID, boolean pIsFirst) {
        return mAutos.followChoreoPath(pPathName, pPID, pIsFirst);
    }

    protected AutoEvent getAuton() {
        return new AutoEvent("EmptyAuto", mAutos, mAutos.getAutoFactory());
    }
}
