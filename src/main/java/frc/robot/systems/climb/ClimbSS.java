// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;
import frc.lib.tuning.LoggedTunableNumber;

public class ClimbSS extends SubsystemBase {

  private static final LoggedTunableNumber tKS = new LoggedTunableNumber("Climb/Control/kS", ClimbConstants.kController.feedforward().getKs());
  private static final LoggedTunableNumber tKV = new LoggedTunableNumber("Climb/Control/kV", ClimbConstants.kController.feedforward().getKv());
  private static final LoggedTunableNumber tKG = new LoggedTunableNumber("Climb/Control/kG", ClimbConstants.kController.feedforward().getKg());
  private static final LoggedTunableNumber tKTolerance = new LoggedTunableNumber("Climb/Control/kTolerance", ClimbConstants.kController.tolerance());


  public enum ClimbGoal {
    kStow(() -> Units.inchesToMeters(0)),
    kLc1(() -> Units.inchesToMeters(0)),
    kLc2(() -> Units.inchesToMeters(0)),
    kLc3(() -> Units.inchesToMeters(0)),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Climb/Custom", 0.0));

    private DoubleSupplier goalMeters;

    ClimbGoal(DoubleSupplier goalMeters) {
      this.goalMeters = goalMeters;
    }

    public double getGoalMeters() {
      return this.goalMeters.getAsDouble();
    }
  }

  private final ClimbIO mClimbIO;
  private final PositionSoftLimits mSoftLimits;
  private final ClimbInputsAutoLogged mClimbInputs = new ClimbInputsAutoLogged();
  private ClimbGoal mCurrentGoal = null;
  private double mCurrentClimbGoalPositionMeters = 0.0;

  private ElevatorFeedforward mFeedforward;
  private BangBangController mController;

  public ClimbSS(ClimbIO pClimbIO, PositionSoftLimits pSoftLimits) {
    mClimbIO = pClimbIO;
    mSoftLimits = pSoftLimits;
    mController = new BangBangController(ClimbConstants.kController.tolerance());
    mFeedforward = new ElevatorFeedforward(mCurrentClimbGoalPositionMeters, mCurrentClimbGoalPositionMeters, mCurrentClimbGoalPositionMeters);
  }
  
  @Override
  public void periodic() {
    mClimbIO.updateInputs(mClimbInputs);
    Logger.processInputs("Climb", mClimbInputs);

    if (DriverStation.isDisabled()){
      stopClimbMotor();
    }

    if (mCurrentGoal != null) {
      mCurrentClimbGoalPositionMeters = mCurrentGoal.getGoalMeters();

      setPosition(mCurrentClimbGoalPositionMeters);
    }
  }
  
  public void setPosition(double pMeters){
    mClimbIO.setMotorVolts(mController.calculate(pMeters) * 12 + mFeedforward.calculate(mClimbInputs.iClimbVelocityMPS));
  }
  
  public void setGoal(ClimbGoal pGoal){
    mCurrentGoal = pGoal;
  }

  public void setClimbVolts(double pVolts) {
    mClimbIO.setMotorVolts(pVolts);
  }

  public void stopClimbMotor() {
    mClimbIO.stopMotor();
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
  }


  @AutoLogOutput(key = "Climb/Goal")
  public ClimbGoal getGoal(){
    return mCurrentGoal;
  }

}
