// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.conveyor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class Conveyor extends SubsystemBase {

  public enum ConveyorGoal {
    kConveyor(() -> 8.0),
    kOuttake(() -> -6.0),
    kStop(() -> 0.0),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Conveyor/Custom", 0.0));

    private DoubleSupplier goalMeters;

    ConveyorGoal(DoubleSupplier goalMeters) {
      this.goalMeters = goalMeters;
    }

    public double getGoalVolts() {
      return this.goalMeters.getAsDouble();
    }
  }


  private final ConveyorIO mConveyorIO;
  private final ConveyorInputsAutoLogged mConveyorInputs = new ConveyorInputsAutoLogged();

  private ConveyorGoal mCurrentGoal = null;
  private double mCurrentConveyorGoalVolts = 0.0;

  /** Creates a new Climb. */
  public Conveyor(ConveyorIO pConveyorIO) {
    this.mConveyorIO = pConveyorIO;
  }
  
  @Override
  public void periodic() {
    mConveyorIO.updateInputs(mConveyorInputs);
    Logger.processInputs("Conveyor", mConveyorInputs);
    
    if(DriverStation.isDisabled()){
      stopConveyorMotor();
    }

    if (mCurrentGoal != null){
      mCurrentConveyorGoalVolts = mCurrentGoal.getGoalVolts();

      setConveyorVolts(mCurrentConveyorGoalVolts);
    }

  }

  public void setConveyorVolts(double pVolts) {
    mConveyorIO.setMotorVolts(pVolts);
  }

  public void stopConveyorMotor() {
    mConveyorIO.stopMotor();
  }

    public void setGoal(ConveyorGoal pGoal){
    mCurrentGoal = pGoal;
  }

  @AutoLogOutput(key = "Conveyor/Goal")
  public ConveyorGoal getGoal(){
    return mCurrentGoal;
  }
  
}