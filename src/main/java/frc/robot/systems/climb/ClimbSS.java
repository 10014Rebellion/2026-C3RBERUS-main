// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;
import frc.lib.tuning.LoggedTunableNumber;

public class ClimbSS extends SubsystemBase {

  public static final LoggedTunableNumber tElevatorCustomSetpointMeters = 
    new LoggedTunableNumber("Climb/Control/CustomSetpointRot", 0);
  
  public enum ClimbState {
    STOW(() -> Units.inchesToMeters(0)),
    MID(() -> Units.inchesToMeters(0)),
    HIGH(() -> Units.inchesToMeters(0)),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    TUNING(() -> tElevatorCustomSetpointMeters.get());
    
    private DoubleSupplier goalMeters;
    
    ClimbState(DoubleSupplier goalMeters) {
      this.goalMeters = goalMeters;
    }
    
    public double getGoalMeters() {
      return this.goalMeters.getAsDouble();
    }
  }
  
  private final ClimbIO mClimbIO;
  private final ClimbInputsAutoLogged mClimbInputs = new ClimbInputsAutoLogged();
  
  private ElevatorFeedforward mFeedforward;
  
  private static final LoggedTunableNumber tKS = new LoggedTunableNumber("Climb/Control/kS", ClimbConstants.kController.feedforward().getKs());
  private static final LoggedTunableNumber tKV = new LoggedTunableNumber("Climb/Control/kV", ClimbConstants.kController.feedforward().getKv());
  private static final LoggedTunableNumber tKG = new LoggedTunableNumber("Climb/Control/kG", ClimbConstants.kController.feedforward().getKg());
  private static final LoggedTunableNumber tKP = new LoggedTunableNumber("Climb/Control/kP", ClimbConstants.kController.pdController().kP());
  private static final LoggedTunableNumber tKD = new LoggedTunableNumber("Climb/Control/kD", ClimbConstants.kController.pdController().kD());

  private ClimbState mClimbGoal = null;
  private Double mCurrentClimbGoalPositionMeters = 0.0;
  private Double mAppliedVolts = null;


  public ClimbSS(ClimbIO pClimbIO, PositionSoftLimits pSoftLimits) {
    mClimbIO = pClimbIO;
    mFeedforward = ClimbConstants.kController.feedforward();
  }

  @Override
  public void periodic() {
    mClimbIO.updateInputs(mClimbInputs);

    refreshTuneables();
    mClimbIO.enforceSoftLimits();

    Logger.processInputs("Climb", mClimbInputs);

    if(mClimbGoal != null) {
      mCurrentClimbGoalPositionMeters = mClimbGoal.getGoalMeters();
      setClimbPositionMeters(mCurrentClimbGoalPositionMeters);
    }
  }

  public Command setClimbStateCmd(ClimbState pState){
    return Commands.run(() -> {
      setClimbState(pState);
    }, this);
  }

  public Command setClimbVoltsCmd(double pVolts){
    return Commands.run(() -> {
      setClimbVolts(pVolts);
    }, this);
  }

  public Command setClimbPositionManualCmd(double pPosition){
    return Commands.run(() -> {
      setClimbMotorManual(pPosition);
    }, this);
  }

  public Command setClimbPositionManualCmd(){
    return Commands.run(() -> {
      setClimbMotorManual();
    }, this);
  }

  public Command stopClimbCmd(){
    return Commands.run(() -> {
      stopClimbMotor();
    }, this);
  }
  
  public void setClimbState(ClimbState pClimbState){
    mAppliedVolts = null;
    mClimbGoal = pClimbState;
  }

  public void setClimbVolts(double pVolts){
    mCurrentClimbGoalPositionMeters = null;
    mAppliedVolts = pVolts;
    mClimbGoal = null;
    mClimbIO.setMotorVolts(mAppliedVolts);
  }

  public void stopClimbMotor(){
    mCurrentClimbGoalPositionMeters = null;
    mAppliedVolts = null;
    mClimbGoal = null;
    mClimbIO.stopMotor();
  }

  public void setClimbMotorManual(double pPositionMeters){
    mAppliedVolts = null;
    mClimbGoal = null;
    setClimbPositionMeters(pPositionMeters);
  }

  public void setClimbMotorManual(){
    mAppliedVolts = null;
    mClimbGoal = null;
    setClimbPositionMeters(tElevatorCustomSetpointMeters.get());
  }

  public void setClimbPositionMeters(double pPosition){
    mClimbIO.setMotorPosition(pPosition, mFeedforward.calculate(mClimbInputs.iClimbVelocityMPS));
  }

  public void setFF(double pKS, double pKG, double pKV){
    mFeedforward.setKs(pKS);
    mFeedforward.setKg(pKG);
    mFeedforward.setKv(pKV);
  }

  public void refreshTuneables(){
    LoggedTunableNumber.ifChanged(
      hashCode(), 
      () -> setFF(tKS.get(), tKG.get(), tKV.get()), 
      tKG, tKS, tKV);

    LoggedTunableNumber.ifChanged(
      hashCode(), 
      () -> mClimbIO.setPDConstants(tKP.get(), tKD.get()), 
      tKP, tKD);

  }
}
