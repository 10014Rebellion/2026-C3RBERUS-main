package frc.robot.systems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.hood.HoodSS;

public class Shooter {
  private final FuelPumpSS mFuelPumpsSS;
  private final HoodSS mHoodSS;
  private final FlywheelsSS mFlywheelSS;

  public Shooter(FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelSS) {
    this.mFuelPumpsSS = pFuelPumpSS; this.mHoodSS = pHoodSS; this.mFlywheelSS = pFlywheelSS; // I miss my C++ initializer lists :'(
  }

  public Command setFlywheelsRPSCmd(Rotation2d pRPS) {
    return new InstantCommand(() -> mFlywheelSS.setFlywheelSpeeds(pRPS), mFlywheelSS);
  }

  public Command setFlywheelsRPSCmd() {
    return new InstantCommand(() -> mFlywheelSS.setFlywheelSpeeds(), mFlywheelSS);
  }

  public Command setFlywheelsVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mFlywheelSS.setFlywheelVolts(pVolts), mFlywheelSS);
  }

  public Command setFuelPumpsVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mFuelPumpsSS.setFuelPumpVolts(pVolts));
  }

  public Command setFuelPumpRPSCmd(double pRPS) {
    return new InstantCommand(() -> mFuelPumpsSS.setFuelPumpRPS(pRPS));
  }

  public Command setHoodRot(Rotation2d pRot) {
    return new InstantCommand(() -> mHoodSS.setHoodRot(pRot));
  }

  public Command holdHoodCmd() {
    return new InstantCommand(() -> mHoodSS.holdHood());
  }

  public Command setHoodVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mHoodSS.setHoodVolts(pVolts));
  }
}
