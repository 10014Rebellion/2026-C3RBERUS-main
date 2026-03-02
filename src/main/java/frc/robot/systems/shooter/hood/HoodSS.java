package frc.robot.systems.shooter.hood;

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
import frc.robot.systems.shooter.ShooterConstants;
import frc.robot.systems.shooter.ShooterConstants.HoodConstants;

public class HoodSS extends SubsystemBase{

  private static final LoggedTunableNumber tHoodCustomSetpoint = 
    new LoggedTunableNumber("Hood/Control/CustomSetpointDegrees", 0);

  @AutoLogOutput (key="Hood/IncrementAngle")
  private static Rotation2d incrementAngle = Rotation2d.kZero; 

  public static enum HoodState {
    MAX(() -> ShooterConstants.HoodConstants.kHoodLimits.forwardLimit()),
    STOWED(() -> Rotation2d.fromDegrees(0)),
    CUSTOM(() -> incrementAngle),
    MID(() -> ShooterConstants.HoodConstants.kHoodLimits.forwardLimit().div(2)),
    MIN(() -> ShooterConstants.HoodConstants.kHoodLimits.backwardLimit());

    private Supplier<Rotation2d> mRSupplier;

    private HoodState(Supplier<Rotation2d> pRSupplier) {
      mRSupplier = pRSupplier;
    }

    public Rotation2d getDesiredRotation() {
      return mRSupplier.get();
    } 
  }
  
  private final HoodIO mHoodIO;
  private final ArmFeedforward mHoodFF;
  private final HoodInputsAutoLogged mHoodInputs = new HoodInputsAutoLogged();

  private final LoggedTunableNumber tHoodKP = new LoggedTunableNumber("Hood/Control/kP", HoodConstants.kHoodControlConfig.pdController().kP());
  private final LoggedTunableNumber tHoodKD = new LoggedTunableNumber("Hood/Control/kD", HoodConstants.kHoodControlConfig.pdController().kD());
  private final LoggedTunableNumber tHoodKS = new LoggedTunableNumber("Hood/Control/kS", HoodConstants.kHoodControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tHoodKG = new LoggedTunableNumber("Hood/Control/kG", HoodConstants.kHoodControlConfig.feedforward().getKg());
  private final LoggedTunableNumber tHoodKV = new LoggedTunableNumber("Hood/Control/kV", HoodConstants.kHoodControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tHoodKA = new LoggedTunableNumber("Hood/Control/kA", HoodConstants.kHoodControlConfig.feedforward().getKa());
  private final LoggedTunableNumber tHoodCruiseVel = new LoggedTunableNumber("Hood/Control/CruiseVel", HoodConstants.kHoodControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tHoodMaxAccel = new LoggedTunableNumber("Hood/Control/MaxAcceleration", HoodConstants.kHoodControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tHoodMaxJerk = new LoggedTunableNumber("Hood/Control/MaxJerk", HoodConstants.kHoodControlConfig.motionMagicConstants().maxJerk());
  private final LoggedTunableNumber tHoodTolerance = new LoggedTunableNumber("Hood/Control/Tolerance", HoodConstants.kToleranceRotations);

  @AutoLogOutput(key="Hood/State")
  private HoodState mHoodState = HoodState.STOWED;

  private Rotation2d mCurrentRotationalGoal = Rotation2d.kZero;
  private Double mAppliedVolts = null;

  public HoodSS(HoodIO pHoodIO) {
    this.mHoodIO = pHoodIO;
    this.mHoodFF = HoodConstants.kHoodControlConfig.feedforward();
  }
  
  @Override
  public void periodic() {
    mHoodIO.updateInputs(mHoodInputs);

    refreshTuneables();
    mHoodIO.enforceSoftLimits();

    Logger.processInputs("Hood", mHoodInputs);

    if(mHoodState != null) {
      mCurrentRotationalGoal = mHoodState.getDesiredRotation();
      setHoodRot(mCurrentRotationalGoal);
    }

    Logger.recordOutput("Hood/EnumVal", mHoodState.getDesiredRotation());
  }

  public Command incrementAngle() {
    return Commands.runOnce(() -> {
      incrementAngle = Rotation2d.fromRotations(MathUtil.clamp(
        incrementAngle.plus(HoodConstants.kIncrementStepAmount).getRotations(), 
        HoodConstants.kHoodLimits.backwardLimit().getRotations(), 
        HoodConstants.kHoodLimits.forwardLimit().getRotations())
      );
    }, this).andThen(setHoodStateCmd(HoodState.CUSTOM));
  }

  public Command decrementAngle() {
    return Commands.runOnce(() -> {
      incrementAngle = Rotation2d.fromRotations(MathUtil.clamp(
        incrementAngle.minus(HoodConstants.kIncrementStepAmount).getRotations(), 
        HoodConstants.kHoodLimits.backwardLimit().getRotations(), 
        HoodConstants.kHoodLimits.forwardLimit().getRotations())
      );
    }, this).andThen(setHoodStateCmd(HoodState.CUSTOM));
  }

  public Command setHoodStateCmd(HoodState pHoodState){
    return Commands.run(() -> {
      setHoodState(pHoodState);
    }, this);
  }

  public Command setHoodVoltsCmd(double pVolts){
    return Commands.run(() -> {
      setHoodVolts(pVolts);
    }, this);
  }

  public Command setHoodRotationManualCmd(Rotation2d pRot){
    return Commands.run(() -> {
      setHoodRotManual(pRot);
    }, this);
  }

  public Command setHoodRotationManualCmd(){
    return Commands.run(() -> {
      setHoodRotManual();
    }, this);
  }

  public Command holdHoodCmd(){
    return Commands.run(() -> {
      setHoodRotManual((mHoodInputs.iHoodAngle));
    }, this);
  }

  public Command stopHoodCmd(){
    return Commands.run(() -> {
      stopHoodMotor();
    }, this);
  }

  public void setHoodState(HoodState pHoodState) {
    mAppliedVolts = null;
    mHoodState = pHoodState;
  }

  public void setHoodVolts(double pVolts) {
    mCurrentRotationalGoal = null;
    mHoodState = null;
    mAppliedVolts = pVolts;
    mHoodIO.setMotorVolts(mAppliedVolts);
  }
  
  public void stopHoodMotor() {
    mCurrentRotationalGoal = null;
    mHoodState = null;
    mAppliedVolts = 0.0;
    mHoodIO.stopMotor();
  }

  public void setHoodRotManual(Rotation2d pRot) {
    mAppliedVolts = null;
    mHoodState = null;
    setHoodRot(pRot);
  }

  public void setHoodRotManual() {
    mAppliedVolts = null;
    mHoodState = null;
    setHoodRot(Rotation2d.fromDegrees(tHoodCustomSetpoint.get()));
  }

  public void setHoodState() {
    mAppliedVolts = null;
    mHoodState = null;
  }

  public void holdHood() {
    mCurrentRotationalGoal = mHoodInputs.iHoodAngle;
    mHoodIO.setMotorPosition(mHoodInputs.iHoodAngle, mHoodInputs.iHoodVelocityRPS);
  }

  public void setHoodRot(Rotation2d pRotSP) {
    mCurrentRotationalGoal = pRotSP;
    mHoodIO.setMotorPosition(mCurrentRotationalGoal, mHoodFF.calculate(mCurrentRotationalGoal.getRadians(), mHoodInputs.iHoodVelocityRPS));
  }
  
  public Rotation2d getPosition(){
    return mHoodInputs.iHoodAngle;
  }

  private void setFF(double kS, double kG, double kV, double kA) {
    mHoodFF.setKs(kS);
    mHoodFF.setKg(kG);
    mHoodFF.setKv(kV);
    mHoodFF.setKa(kA);
  }
  
  @AutoLogOutput(key = "Shooter/Hood/Feedback/ErrorRotation")
  public double getErrorPositionRotations() {
    return mCurrentRotationalGoal.getRotations() - getPosition().getRotations();
  }

  @AutoLogOutput(key = "Shooter/Hood/Feedback/CurrentGoal")
  public Rotation2d getCurrentGoal() {
    return mCurrentRotationalGoal;
  }

  @AutoLogOutput(key = "Shooter/Hood/Feedback/AtGoal")
  public boolean atGoal() {
    return Math.abs(getErrorPositionRotations()) < tHoodTolerance.get();
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
