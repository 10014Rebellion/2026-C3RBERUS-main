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

  public static final LoggedTunableNumber tFlywheelCustomSetpointRPS = 
    new LoggedTunableNumber("Flywheel/Control/CustomSetpointRPS", 0);

  public static enum FlywheelState {
    SHOOT_FAR(() -> Rotation2d.fromRotations(90)),
    STANDBY(() -> Rotation2d.fromRotations(20)),
    SHOOT_CLOSE(() -> Rotation2d.fromRotations(70)),
    IDLE(() -> Rotation2d.fromRotations(0)),
    TUNING(() -> Rotation2d.fromRotations(tFlywheelCustomSetpointRPS.getAsDouble()));

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

  private final LoggedTunableNumber tFlywheelTolerance = new LoggedTunableNumber("Flywheel/Control/Tolerance", ShooterConstants.FlywheelConstants.kToleranceRPS);

  private Rotation2d mCurrentRPSGoal = Rotation2d.kZero;
  private FlywheelState mFlywheelState = null;

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
      if(mFlywheelState.equals(FlywheelState.IDLE)) setFlywheelVolts(0.0);
       else {Logger.recordOutput("Flywheel/State", mFlywheelState);
      setFlywheelClosedLoop(mFlywheelState.getDesiredRotation()); }
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

  public Command setFlywheelAmpsCmd(double pAmps){
    return Commands.run(() -> {
      setFlywheelAmps(pAmps);
    }, this);
  }

  public Command setFlywheelsVelocityManualCmd(Rotation2d pRotpS){
    return Commands.run(() -> {
      setFlywheelVelocityManual(pRotpS);
    }, this);
  }

  public Command stopFlywheelCmd(){
    return Commands.run(() -> {
      stopFlywheels();
    }, this);
  }

  public void setFlywheelState(FlywheelState mState) {
    mFlywheelState = mState;
  }

  public void setFlywheelVolts(double pVolts) {
    mFlywheelState = null;
    mLeaderFlywheelIO.setMotorVolts(pVolts);
    mFollowerFlywheelIO.enforceFollower();
  }

  public void setFlywheelAmps(double pVolts) {
    mFlywheelState = null;
    mLeaderFlywheelIO.setMotorAmps(pVolts);
    mFollowerFlywheelIO.enforceFollower();
  }

  public void setFlywheelVelocityManual(Rotation2d pRotsPerS){
    mFlywheelState = null;
    setFlywheelClosedLoop(pRotsPerS);
  }

  public void stopFlywheels(){
    mFlywheelState = null;
    mLeaderFlywheelIO.stopMotor();
    mFollowerFlywheelIO.enforceFollower();
  }

  public void setFlywheelClosedLoop(Rotation2d pRotsPerS){
    mCurrentRPSGoal = pRotsPerS;
    Logger.recordOutput("Flywheel/Sigma", mCurrentRPSGoal);
    mLeaderFlywheelIO.setMotorVel(
      pRotsPerS.getRotations(),
      0.0);
    mFollowerFlywheelIO.enforceFollower();
  }

  public Rotation2d getFlywheelRPS() {
    return mEncoderInputs.iEncoderVelocityRPS;
  }

  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFlywheelIO.setPDConstants(pKP, pKD);
    mFollowerFlywheelIO.setPDConstants(pKP, pKD);
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothPDConstants(tFlywheelKP.get(), tFlywheelKD.get()), 
      tFlywheelKP, tFlywheelKD
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
