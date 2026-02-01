// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.HardwareRecords.ElevatorController;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;
import frc.lib.tuning.LoggedTunableNumber;

public class Climb extends SubsystemBase {

  @AutoLogOutput(key = "Elevator/CurrentSlot")
  public static int mSlot = 0;

  // private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/" + mSlot + "/mSlot/P", kControllers.getSlot(mSlot, ElevatorController.class).pdController().kP());
  // private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/" + mSlot + "/mSlot/D", kControllers.getSlot(mSlot, ElevatorController.class).pdController().kD());
  // private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/" + mSlot + "/mSlot/S", kControllers.getSlot(mSlot, ElevatorController.class).feedforward().getKs());
  // private static final LoggedTunableNumber kMaxV = new LoggedTunableNumber("Elevator/" + mSlot + "/mSlot/V", kControllers.getSlot(mSlot, ElevatorController.class).feedforward().getKg());
  // private static final LoggedTunableNumber kMaxA = new LoggedTunableNumber("Elevator/" + mSlot + "/mSlot/G", kControllers.getSlot(mSlot, ElevatorController.class).feedforward().getKa());
  // private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/" + mSlot + "/mSlot/MaxVelo", kControllers.getSlot(mSlot, ElevatorController.class).constraints().maxVelocity());
  // private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/" + mSlot + "/mSlot/MaxAccel", kControllers.getSlot(mSlot, ElevatorController.class).constraints().maxAcceleration());
  // private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/" + mSlot + "/mSlot/MaxJerk", kControllers.getSlot(mSlot, ElevatorController.class).constraints().maxJerk());

  public enum ClimbGoal {
    kStow(() -> Units.inchesToMeters(60.0)),
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

  public Climb(ClimbIO pClimbIO, PositionSoftLimits pSoftLimits) {
    this.mClimbIO = pClimbIO;
    this.mSoftLimits = pSoftLimits;
    // this.mFeedforward = kControllers.getSlot(mSlot, ElevatorFeedforward.class);
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

      setPosition(mSlot, mCurrentClimbGoalPositionMeters);
    }
  }

  public void setClimbVolts(double pVolts) {

    if (mClimbInputs.iClimbMotorVolts > 0 && mClimbInputs.iClimbPositionMeters > mSoftLimits.forwardLimitM()){return;}

    else if (mClimbInputs.iClimbMotorVolts < 0 && mClimbInputs.iClimbPositionMeters < mSoftLimits.backwardLimitM()){return;}

    mClimbIO.setMotorVolts(pVolts);
  }

  public void stopClimbMotor() {
    mClimbIO.stopMotor();
  }

  public void setGoal(ClimbGoal pGoal){
    mCurrentGoal = pGoal;
  }

  public void setPosition(int pSlot, double pMeters){
    mClimbIO.setMotorPosition(pSlot, pMeters, mFeedforward.calculate(mClimbInputs.iClimbVelocityMPS, mClimbInputs.iClimbAccelerationMPSS));
  }

  public void setFeedForwardConstants(int pSlot, double pKS, double pKV, double pKG){

  }

  @AutoLogOutput(key = "Climb/Goal")
  public ClimbGoal getGoal(){
    return mCurrentGoal;
  }

}
