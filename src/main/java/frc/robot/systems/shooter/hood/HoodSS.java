package frc.robot.systems.shooter.hood;

import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class HoodSS extends SubsystemBase{
  public static enum HoodStates {
    STOPPED, // At rest
    OPEN_LOOP, // Via voltage out
    DEFINED_SETPOINT, // Going to a setpoint from the closed loop enums
    CONSTANT_SETPOINT, // Going to a setpoint passed in as a number
    CUSTOM_SETPOINT, // Via increment/decrement
    TUNEABLE_SETPOINT // Via NT
  }
  
  public static enum HoodClosedSetpoints {
    MAX(() -> HoodConstants.kHoodLimits.forwardLimit()),
    MID(() -> HoodConstants.kHoodLimits.forwardLimit().div(2)),
    CLOSE_SHOT(() -> Rotation2d.fromDegrees(5.0)),
    BUMP_SHOT(() -> Rotation2d.fromDegrees(10.0)),
    MIN(() -> HoodConstants.kHoodLimits.backwardLimit());
    
    private Supplier<Rotation2d> mRotSupplier;
    
    private HoodClosedSetpoints(Supplier<Rotation2d> pRotSupplier) {
      this.mRotSupplier = pRotSupplier;
    }
    
    public Rotation2d getRot() {
      return mRotSupplier.get();
    } 
  }
  
  private final HoodIO mHoodIO;
  private final ArmFeedforward mHoodFF;
  private final HoodInputsAutoLogged mHoodInputs = new HoodInputsAutoLogged();
  
  private final LoggedTunableNumber tHoodKP = new LoggedTunableNumber("Hood/PID/kP", HoodConstants.kHoodControlConfig.pdController().kP());
  private final LoggedTunableNumber tHoodKD = new LoggedTunableNumber("Hood/PID/kD", HoodConstants.kHoodControlConfig.pdController().kD());
  private final LoggedTunableNumber tHoodKS = new LoggedTunableNumber("Hood/FF/kS", HoodConstants.kHoodControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tHoodKG = new LoggedTunableNumber("Hood/FF/kG", HoodConstants.kHoodControlConfig.feedforward().getKg());
  private final LoggedTunableNumber tHoodKV = new LoggedTunableNumber("Hood/FF/kV", HoodConstants.kHoodControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tHoodKA = new LoggedTunableNumber("Hood/FF/kA", HoodConstants.kHoodControlConfig.feedforward().getKa());
  private final LoggedTunableNumber tHoodCruiseVel = new LoggedTunableNumber("Hood/Profile/CruiseVel", HoodConstants.kHoodControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tHoodMaxAccel = new LoggedTunableNumber("Hood/Profile/MaxAcceleration", HoodConstants.kHoodControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tHoodMaxJerk = new LoggedTunableNumber("Hood/Profile/MaxJerk", HoodConstants.kHoodControlConfig.motionMagicConstants().maxJerk());
  private final LoggedTunableNumber tHoodTolerance = new LoggedTunableNumber("Hood/Control/Tolerance", HoodConstants.kToleranceRotations);
  private static final LoggedTunableNumber tHoodTuneableSetpointDeg = new LoggedTunableNumber("Hood/Control/TuneableSetpointDegrees", 0);
  
  @AutoLogOutput(key = "Shooter/Hood/States/CurrentState")
  private HoodStates mCurrentHoodState = HoodStates.STOPPED;

  private Optional<Rotation2d> oGoalAngle = Optional.empty();
  
  public HoodSS(HoodIO pHoodIO) {
    this.mHoodIO = pHoodIO;
    this.mHoodFF = HoodConstants.kHoodControlConfig.feedforward();

    
    this.setDefaultCommand(
      Commands.run(() -> {
        if (oGoalAngle.isPresent()) {
          mHoodIO.setMotorPosition(oGoalAngle.get(), mHoodFF.calculate(mHoodInputs.iHoodAngle.getRadians(), 0.0)); // closed loop, holds with kG
        } else {
          mHoodIO.setMotorVolts(0.0); // open loop stop
          mCurrentHoodState = HoodStates.STOPPED;
        }
      }, this)
    );
  }

  public HoodStates getHoodState() {
    return mCurrentHoodState;
  }

  public void clearGoalWithoutStateChange() {
    this.oGoalAngle = Optional.empty();
  }

  private void clearGoal(HoodStates pNewState) {
    this.oGoalAngle = Optional.empty();
    this.mCurrentHoodState = pNewState;
  }

  public void setGoal(Rotation2d pGoal, HoodStates pNewState) {
    this.oGoalAngle = Optional.of(clampRotToSoftLimits(pGoal));
    this.mCurrentHoodState = pNewState;
  }

  public Command setGoalCmd(Rotation2d pGoal) {
    return Commands.runOnce(() -> setGoal(pGoal, HoodStates.CONSTANT_SETPOINT), this);
  }

  public Command setGoalCmd(HoodClosedSetpoints pHoodSetpoint) {
    return Commands.runOnce(() -> setGoal(pHoodSetpoint.getRot(), HoodStates.DEFINED_SETPOINT), this);
  }

  public Command incrementAngleCmd() {
    return Commands.runOnce(() -> setGoal(mHoodInputs.iHoodAngle.plus(HoodConstants.kIncrementStepAmount), HoodStates.CUSTOM_SETPOINT), this);
  }

  public Command decrementAngleCmd() {
    return Commands.runOnce(() -> setGoal(mHoodInputs.iHoodAngle.minus(HoodConstants.kIncrementStepAmount), HoodStates.CUSTOM_SETPOINT), this);
  }

  public Command setHoodVoltsCmdNoEnd(double pVolts){
    return Commands.runOnce(() -> clearGoal(HoodStates.OPEN_LOOP))
      .andThen(Commands.run(() -> mHoodIO.setMotorVolts(pVolts), this));
  }

  public Command setHoodTuneableCmdNoEnd() {
    return Commands.run(() -> {
      setGoal(Rotation2d.fromDegrees(tHoodTuneableSetpointDeg.getAsDouble()), HoodStates.TUNEABLE_SETPOINT);
      mHoodIO.setMotorPosition(getCurrentGoal(), mHoodFF.calculate(mHoodInputs.iHoodAngle.getRadians(), 0.0));
    }, this);
  }

  private void setFF(double kS, double kG, double kV, double kA) {
    mHoodFF.setKs(kS);
    mHoodFF.setKg(kG);
    mHoodFF.setKv(kV);
    mHoodFF.setKa(kA);
  }
  
  @AutoLogOutput(key = "Shooter/Hood/Feedback/ErrorRotation")
  public double getErrorPositionRotations() {
    return getCurrentGoal().getRotations() - mHoodInputs.iHoodAngle.getRotations();
  }

  @AutoLogOutput(key = "Shooter/Hood/Feedback/CurrentGoal")
  public Rotation2d getCurrentGoal() {
    return oGoalAngle.orElse(mHoodInputs.iHoodAngle);
  }

  @AutoLogOutput(key = "Shooter/Hood/Feedback/HasGoal")
  public boolean hasGoal() {
    return oGoalAngle.isPresent();
  }

  @AutoLogOutput(key = "Shooter/Hood/Feedback/AtGoal")
  public boolean atGoal() {
    return hasGoal() && Math.abs(getErrorPositionRotations()) < tHoodTolerance.get();
  }

  private Rotation2d clampRotToSoftLimits(Rotation2d pRotToClamp) {
    return Rotation2d.fromRotations(
      MathUtil.clamp(
        pRotToClamp.getRotations(),
        HoodConstants.kHoodLimits.backwardLimit().getRotations(),
        HoodConstants.kHoodLimits.forwardLimit().getRotations()
      )
    );
  }

  @Override
  public void periodic() {
    mHoodIO.updateInputs(mHoodInputs);

    refreshTuneables();
    Logger.recordOutput("Hood/Debugging/TuneableSetpointDeg", tHoodTuneableSetpointDeg.getAsDouble());

    Logger.processInputs("Hood", mHoodInputs);
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mHoodIO.setPDConstants(tHoodKP.get(), tHoodKD.get()), 
      tHoodKP, tHoodKD
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setFF(tHoodKS.get(), tHoodKG.get(), tHoodKV.get(), tHoodKA.get()), 
      tHoodKS, tHoodKG, tHoodKV, tHoodKA
    );
  
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mHoodIO.setMotionMagicConstants(tHoodCruiseVel.get(), tHoodMaxAccel.get(), tHoodMaxJerk.get()), 
      tHoodCruiseVel, tHoodMaxAccel, tHoodMaxJerk
    );
  }
}
