package frc.robot.systems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelState;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodSS.HoodState;

public class Shooter {
  private final FuelPumpSS mFuelPumpsSS;
  private final HoodSS mHoodSS;
  private final FlywheelsSS mFlywheelSS;

  public Shooter(FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelSS) {
    this.mFuelPumpsSS = pFuelPumpSS; this.mHoodSS = pHoodSS; this.mFlywheelSS = pFlywheelSS; // I miss my C++ initializer lists :'(
  }

  public Command setFlywheelStateCmd(FlywheelState pState){
    return mFlywheelSS.setFlywheelStateCmd(pState);
  }

  public Command setFlywheelRPSCmd(Rotation2d pRPS) {
    return mFlywheelSS.setFlywheelsRPSManualCmd(pRPS);
  }

  public Command setFlywheelsRPSCmd() {
    return mFlywheelSS.setFlywheelsRPSManualCmd();
  }

  public Command setFlywheelsVoltsCmd(double pVolts) {
    return mFlywheelSS.setFlywheelVoltsCmd(pVolts);
  }

  public Command stopFlywheelCmd(){
    return mFlywheelSS.stopFlywheelCmd();
  }

  public Command setFuelPumpState(FuelPumpState pState){
    return mFuelPumpsSS.setFuelPumpStateCmd(pState);
  }

  public Command setFuelPumpVoltsCmd(double pVolts){
    return mFuelPumpsSS.setFuelPumpManualCmd(pVolts);
  }

  public Command stopFuelPumpCmd(){
    return mFuelPumpsSS.stopFuelPumpCmd();
  }

  public Command setHoodStateCmd(HoodState pState) {
    return mHoodSS.setHoodStateCmd(pState);
  }

  public Command setHoodRotationCmd(Rotation2d pRot){
    return mHoodSS.setHoodRotationManualCmd(pRot);
  }

  public Command setHoodRotationCmd(){
    return mHoodSS.setHoodRotationManualCmd();
  }

  public Command setHoodVoltsCmd(double pVolts){
    return mHoodSS.setHoodVoltsCmd(pVolts);
  }

  public Command setHoldHoodCmd(){
    return mHoodSS.holdHoodCmd();
  }

  public Command stopHoodCmd(){
    return mHoodSS.stopHoodCmd();
  }
}
