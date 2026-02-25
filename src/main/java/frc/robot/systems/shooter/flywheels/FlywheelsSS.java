package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.shooter.ShooterConstants;
import frc.robot.systems.shooter.flywheels.encoder.EncoderIO;
import frc.robot.systems.shooter.flywheels.encoder.EncoderInputsAutoLogged;
import static frc.robot.systems.shooter.ShooterConstants.FlywheelConstants.kFlywheelControlConfig;

import java.util.function.Supplier;

public class FlywheelsSS extends SubsystemBase {

  private static final LoggedTunableNumber tFlywheelCustomSetpointRPS = 
    new LoggedTunableNumber("Flywheel/Control/CustomSetpointRPS", 0);

  public static enum FlywheelState {
    SHOOT_FAR(() -> Rotation2d.fromRotations(90)),
    IDLING(() -> Rotation2d.fromRotations(20)),
    SHOOT_CLOSE(() -> Rotation2d.fromRotations(70)),
    IDLE(() -> Rotation2d.fromRotations(0));

    private Supplier<Rotation2d> mRPSSupplier;

    private FlywheelState(Supplier<Rotation2d> pRPSupplier) {
      mRPSSupplier = pRPSupplier;
    }

    public Rotation2d getDesiredRotation() {
      return mRPSSupplier.get();
    } 
  }


  private final FlywheelIO mLeaderFlywheelIO;
  private final FlywheelIO mFollowerFlywheelIO;
  private final EncoderIO mFlywheelEncoder;

  private final FlywheelInputsAutoLogged mLeaderFlywheelInputs = new FlywheelInputsAutoLogged();
  private final FlywheelInputsAutoLogged mFollowerFlywheelInputs = new FlywheelInputsAutoLogged();
  private final EncoderInputsAutoLogged mEncoderInputs = new EncoderInputsAutoLogged();

  private final LoggedTunableNumber tFlywheelKP = new LoggedTunableNumber("Flywheel/Control/kP", kFlywheelControlConfig.pdController().kP());
  private final LoggedTunableNumber tFlywheelKD = new LoggedTunableNumber("Flywheel/Control/kD", kFlywheelControlConfig.pdController().kD());

  private final LoggedTunableNumber tFlywheelKS = new LoggedTunableNumber("Flywheel/Control/kS", kFlywheelControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFlywheelKV = new LoggedTunableNumber("Flywheel/Control/kV", kFlywheelControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFlywheelKA = new LoggedTunableNumber("Flywheel/Control/kA", kFlywheelControlConfig.feedforward().getKa());

  private final LoggedTunableNumber tFlywheelCruiseVel = new LoggedTunableNumber("Flywheel/Control/CruiseVel", kFlywheelControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tFlywheelMaxAccel = new LoggedTunableNumber("Flywheel/Control/MaxAcceleration", kFlywheelControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tFlywheelMaxJerk = new LoggedTunableNumber("Flywheel/Control/MaxJerk", kFlywheelControlConfig.motionMagicConstants().maxJerk());

  private final LoggedTunableNumber tFlywheelTolerance = new LoggedTunableNumber("Flywheel/Control/Tolerance", ShooterConstants.FlywheelConstants.kToleranceRPS);

  private Rotation2d mCurrentRPSGoal = Rotation2d.kZero;
  private FlywheelState mFlywheelState = FlywheelState.IDLE;
  private Double mAppliedVolts = null;

  public FlywheelsSS(FlywheelIO pLeaderFlywheelIO, FlywheelIO pFollowerFlywheelIO, EncoderIO pFlywheelEncoder) {
    this.mLeaderFlywheelIO = pLeaderFlywheelIO;
    this.mFollowerFlywheelIO = pFollowerFlywheelIO;
    this.mFlywheelEncoder = pFlywheelEncoder;
  }
  
  @Override
  public void periodic() {
    mLeaderFlywheelIO.updateInputs(mLeaderFlywheelInputs);
    mFollowerFlywheelIO.updateInputs(mFollowerFlywheelInputs);
    mFlywheelEncoder.updateInputs(mEncoderInputs);
    
    refreshTuneables();
    
    Logger.processInputs("Flywheel/Leader", mLeaderFlywheelInputs);
    Logger.processInputs("Flywheel/Follower", mFollowerFlywheelInputs);
    Logger.processInputs("Flywheel/Encoder", mEncoderInputs);

    if(mFlywheelState != null) {
      mCurrentRPSGoal = mFlywheelState.getDesiredRotation();
      setFlywheelClosedLoop(mCurrentRPSGoal);
    }
    
    // Help conserve some power on the motor), since its a 1 way bearing
    if(mLeaderFlywheelInputs.iFlywheelMotorVolts < 0){
      setFlywheelVolts(0.0);
    } else {
      mFollowerFlywheelIO.enforceFollower(); // Sometimes during enable and disable it no longer follows briefly, this scares me, so this is a failsafe
    }

  }

  public Command setFlywheelStateCmd(FlywheelState pFlywheelState){
    return Commands.run(() -> {
      setFlywheelState(pFlywheelState);
    }, this);
  }

  public Command setFlywheelVoltsCmd(double pVolts){
    return Commands.run(() -> {
      setFlywheelVolts(pVolts);
    }, this);
  }

  public Command setFlywheelsRPSManualCmd(Rotation2d pRotpS){
    return Commands.run(() -> {
      setFlywheelStateManual(pRotpS);
    }, this);
  }

  public Command setFlywheelsRPSManualCmd(){
    return Commands.run(() -> {
      setFlywheelStateManual();
    }, this);
  }

  public Command stopFlywheelCmd(){
    return Commands.run(() -> {
      stopFlywheels();
    }, this);
  }

  public void setFlywheelVolts(double pVolts) {
    mCurrentRPSGoal = null;
    mFlywheelState = null;
    mAppliedVolts = pVolts;
    mLeaderFlywheelIO.setMotorVolts(mAppliedVolts);
    mFollowerFlywheelIO.enforceFollower();
  }

  public void setFlywheelState(FlywheelState mState){
    mCurrentRPSGoal = mState.getDesiredRotation();
    mFlywheelState = mState;
    mAppliedVolts = null;
    setFlywheelClosedLoop(mCurrentRPSGoal);
  }

  public void setFlywheelStateManual(Rotation2d pRotsPerS){
    mCurrentRPSGoal = null;
    mFlywheelState = null;
    mAppliedVolts = null;
    setFlywheelClosedLoop(pRotsPerS);
  }

  public void setFlywheelStateManual(){
    mCurrentRPSGoal = null;
    mFlywheelState = null;
    mAppliedVolts = null;
    setFlywheelClosedLoop(Rotation2d.fromRotations(tFlywheelCustomSetpointRPS.get()));
  }

  public void stopFlywheels(){
    mCurrentRPSGoal = null;
    mFlywheelState = null;
    mAppliedVolts = null;
    mLeaderFlywheelIO.stopMotor();
    mFollowerFlywheelIO.enforceFollower();
  }

  public void setFlywheelClosedLoop(Rotation2d pRotsPerS){
    mLeaderFlywheelIO.setMotorVelAndAccel(
      pRotsPerS.getRotations(), 
      0, 
      kFlywheelControlConfig.feedforward().calculate(pRotsPerS.getRotations()));
    mFollowerFlywheelIO.enforceFollower();
  }

  public Rotation2d getFlywheelRPS() {
    return mEncoderInputs.iEncoderVelocityRPS;
  }

  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFlywheelIO.setPDConstants(pKP, pKD);
    mFollowerFlywheelIO.setPDConstants(pKP, pKD);
  }

  private void setBothMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
    mLeaderFlywheelIO.setMotionMagicConstants(pCruiseVel, pMaxAccel, pMaxJerk);
    mFollowerFlywheelIO.setMotionMagicConstants(pCruiseVel, pMaxAccel, pMaxJerk);
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothPDConstants(tFlywheelKP.get(), tFlywheelKD.get()), 
      tFlywheelKP, tFlywheelKD
    );
  
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothMotionMagicConstants(tFlywheelCruiseVel.get(), tFlywheelMaxAccel.get(), tFlywheelMaxJerk.get()), 
      tFlywheelCruiseVel, tFlywheelMaxAccel, tFlywheelMaxJerk
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setFF(tFlywheelKS.get(), tFlywheelKV.get(), tFlywheelKA.get()),
      tFlywheelKS, tFlywheelKV, tFlywheelKA
    );
  }

  private void setFF(double pKS, double pKV, double pKA){
    kFlywheelControlConfig.feedforward().setKv(pKV);
    kFlywheelControlConfig.feedforward().setKs(pKS);
    kFlywheelControlConfig.feedforward().setKa(pKA);
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/ErrorRotationsPerSec")
  public double getErrorRotationsPerSec() {
    return mCurrentRPSGoal.minus(getFlywheelRPS()).getRotations();
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/CurrentGoal")
  public Rotation2d getCurrentGoal() {
    return mCurrentRPSGoal;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/AtGoal")
  public boolean atGoal() {
    return Math.abs(getErrorRotationsPerSec()) < tFlywheelTolerance.get();
  }
  


}
