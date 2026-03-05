// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

  private final Servo mServo;

  public ClimbSS(ClimbIO pClimbIO, PositionSoftLimits pSoftLimits) {
    mClimbIO = pClimbIO;
    mServo = new Servo(ClimbConstants.kHookPort);
  }

  @Override
  public void periodic() {
    mClimbIO.updateInputs(mClimbInputs);

    // mClimbIO.enforceSoftLimits();

    double currentPosition = mClimbInputs.iClimbPositionMeters;
    if((currentPosition > ClimbConstants.kSoftLimits.forwardLimitM() && mClimbInputs.iClimbMotorVolts > 0) || 
       (currentPosition < ClimbConstants.kSoftLimits.backwardLimitM() && mClimbInputs.iClimbMotorVolts < 0)) mClimbIO.stopMotor();

    Logger.processInputs("Climb", mClimbInputs);
  }

  public Command setClimbVoltsCmd(double pVolts){
    return Commands.run(() -> {
      setClimbVolts(pVolts);
    }, this);
  }

  public Command setClimbState(ClimbState pState){
    return setClimbPositionManualCmd(pState.getGoalMeters());
  }

  public FunctionalCommand setClimbPositionManualCmd(double pPosition) {
    return new FunctionalCommand(
      () -> {},
      () -> {
        if (getClimbPosition() < pPosition)
          setClimbVolts(5.0);

        if (getClimbPosition() > pPosition)
          setClimbVolts(0);
        
      }, (interrupted) -> {
        setClimbVolts(0.0);
      }, 
      () -> atGoal(pPosition));
  }

  public Command stopClimbCmd(){
    return Commands.run(() -> {
      stopClimbMotor();
    }, this);
  }

  public Command unHookClawsCmd(){
    return Commands.run(() -> {
      mServo.setAngle(ClimbConstants.kHookOutPosition);
    }, this);
  }

  public Command hookClawsCmd(){
    return Commands.run(() -> {
      mServo.setAngle(ClimbConstants.kHookInPosition);
    }, this);
  }

  public void stopClimbMotor(){
    mClimbIO.stopMotor();
  }

  public void setClimbVolts(double pVolts){
    mClimbIO.setMotorVolts(pVolts);
  }

  public double getClimbPosition(){
    return mClimbInputs.iClimbPositionMeters;
  }

  public boolean atGoal(double pGoal){
    return Math.abs(pGoal - getClimbPosition()) < 0.01;
  }
}
