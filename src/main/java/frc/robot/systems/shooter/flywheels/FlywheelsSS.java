package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.logging.InvalidValueErrors.UnaccountedEnum;
import frc.robot.systems.shooter.ShotMap;
import frc.robot.systems.shooter.flywheels.encoder.EncoderIO;
import frc.robot.systems.shooter.flywheels.encoder.EncoderInputsAutoLogged;

import static frc.robot.systems.shooter.flywheels.FlywheelConstants.kFlywheelControlConfig;


public class FlywheelsSS extends SubsystemBase {
  // Behavior id for telemetry and restore semantics.

  private final FlywheelIO mLeaderFlywheelIO;
  private final FlywheelIO mFollowerFlywheelIO;
  private final EncoderIO mFlywheelEncoder;

  private final FlywheelInputsAutoLogged mLeaderFlywheelInputs = new FlywheelInputsAutoLogged();
  private final FlywheelInputsAutoLogged mFollowerFlywheelInputs = new FlywheelInputsAutoLogged();
  private final EncoderInputsAutoLogged mEncoderInputs = new EncoderInputsAutoLogged();

  private final LoggedTunableNumber tFlywheelKP = new LoggedTunableNumber("Shooter/Flywheel/Control/PID/kP", kFlywheelControlConfig.pdController().kP());
  private final LoggedTunableNumber tFlywheelKD = new LoggedTunableNumber("Shooter/Flywheel/Control/PID/kD", kFlywheelControlConfig.pdController().kD());
  private final LoggedTunableNumber tFlywheelKS = new LoggedTunableNumber("Shooter/Flywheel/Control/FF/kS", kFlywheelControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFlywheelKV = new LoggedTunableNumber("Shooter/Flywheel/Control/FF/kV", kFlywheelControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFlywheelKA = new LoggedTunableNumber("Shooter/Flywheel/Control/FF/kA", kFlywheelControlConfig.feedforward().getKa());
  private final LoggedTunableNumber tFlywheelMaxVel = new LoggedTunableNumber("Shooter/Flywheel/Control/Profile/MaxVel", kFlywheelControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tFlywheelMaxAccel = new LoggedTunableNumber("Shooter/Flywheel/Control/Profile/MaxAccel", kFlywheelControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tFlywheelMaxJerk = new LoggedTunableNumber("Shooter/Flywheel/Control/Profile/MaxJerk", kFlywheelControlConfig.motionMagicConstants().maxJerk());
  private final LoggedTunableNumber tFlywheelTolerance = new LoggedTunableNumber("Shooter/Flywheel/Control/Tolerance", FlywheelConstants.kToleranceRPS);

  private Rotation2d mLastestClosedLoopGoalRPS = Rotation2d.kZero;

  @AutoLogOutput(key = "Shooter/Flywheel/States/CurrentState")
  private String mCurrentFlywheelBehavior = "STOPPED";

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
  // Behavior commands manage outputs; no enum-driven execute loop.
    
    Logger.processInputs("Shooter/Flywheel/Leader", mLeaderFlywheelInputs);
    Logger.processInputs("Shooter/Flywheel/Follower", mFollowerFlywheelInputs);
    Logger.processInputs("Shooter/Flywheel/Encoder", mEncoderInputs);
  }

  // Per-behavior command factories

  private void stopFlywheels() {
    mLeaderFlywheelIO.stopMotor();
    mFollowerFlywheelIO.enforceFollower();
  }

  private void setFlywheelVoltage(double pVoltage) {
    mLeaderFlywheelIO.setMotorVolts(pVoltage);
    mFollowerFlywheelIO.enforceFollower();
  }

  private void setFlywheelVelocity(Rotation2d pRotsPerS){
    mLastestClosedLoopGoalRPS = pRotsPerS;
    Logger.recordOutput("Shooter/Flywheel/Control/FunctionSetpoint", mLastestClosedLoopGoalRPS);
    mLeaderFlywheelIO.setMotorVelAndAccel(
      pRotsPerS.getRotations(),
      0.0,
      kFlywheelControlConfig.feedforward().calculate(getFlywheelRPS().getRotations()));
    mFollowerFlywheelIO.enforceFollower();
  }

  // Public factories
  public Command stoppedCmd() {
    return Commands.startEnd(() -> { mCurrentFlywheelBehavior = "STOPPED"; stopFlywheels(); }, () -> {}, this);
  }

  public Command standbyVoltageCmd() {
    return Commands.startEnd(() -> { mCurrentFlywheelBehavior = "STANDBY_VOLTAGE"; setFlywheelVoltage(FlywheelConstants.tStandbyVoltage.get()); }, () -> {}, this);
  }

  public Command tuningVoltageCmd() {
    return Commands.startEnd(() -> { mCurrentFlywheelBehavior = "TUNING_VOLTAGE"; setFlywheelVoltage(FlywheelConstants.tTuningVoltage.get()); }, () -> {}, this);
  }

  public Command maxVoltageCmd() {
    return Commands.startEnd(() -> { mCurrentFlywheelBehavior = "MAX_VOLTAGE"; setFlywheelVoltage(FlywheelConstants.tMaxVoltage.get()); }, () -> {}, this);
  }

  private Command makeVelocityCmd(String id, LoggedTunableNumber velocity) {
    return Commands.startEnd(() -> { mCurrentFlywheelBehavior = id; setFlywheelVelocity(Rotation2d.fromRotations(velocity.get())); }, () -> {}, this);
  }

  public Command tuningVelocityCmd() { return makeVelocityCmd("TUNING_VELOCITY", FlywheelConstants.tTuningVelocity); }
  public Command feedVelocityCmd() { return makeVelocityCmd("FEED_VELOCITY", FlywheelConstants.tFeedVelocity); }
  public Command opponentFeedVelocityCmd() { return makeVelocityCmd("OPPONENT_FEED_VELOCITY", FlywheelConstants.tOpponentFeedVelocity); }
  public Command closeVelocityCmd() { return makeVelocityCmd("CLOSE_VELOCITY", FlywheelConstants.tCloseVelocity); }
  public Command towerVelocityCmd() { return makeVelocityCmd("TOWER_VELOCITY", FlywheelConstants.tTowerVelocity); }
  public Command bumpVelocityCmd() { return makeVelocityCmd("BUMP_VELOCITY", FlywheelConstants.tBumpVelocity); }
  public Command maxVelocityCmd() { return makeVelocityCmd("MAX_VELOCITY", FlywheelConstants.tMaxVelocity); }

  public Command shotmapVelocityCmd() {
    return Commands.startEnd(() -> { mCurrentFlywheelBehavior = "SHOTMAP_VELOCITY"; setFlywheelVelocity(ShotMap.getInstance().getFlywheelVel()); }, () -> {}, this);
  }

  public Rotation2d getFlywheelRPS() {
    return mEncoderInputs.iEncoderVelocityRPS;
  }

  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFlywheelIO.setPDConstants(pKP, pKD);
    mFollowerFlywheelIO.setPDConstants(pKP, pKD);
  }

  private void setFF(double pKS, double pKV, double pKA){
    kFlywheelControlConfig.feedforward().setKv(pKV);
    kFlywheelControlConfig.feedforward().setKs(pKS);
    kFlywheelControlConfig.feedforward().setKa(pKA);
  }

  public String getFlywheelState() { return mCurrentFlywheelBehavior; }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/ErrorRPS")
  public double getErrorRPS() {
    return mLastestClosedLoopGoalRPS.minus(mLeaderFlywheelInputs.iFlywheelRotorVelocityRPS).getRotations();
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/LatestClosedLoopGoalRPS")
  public Rotation2d getLatestClosedLoopGoal() {
    return mLastestClosedLoopGoalRPS;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/AtLatestClosedLoopGoal")
  public boolean atLatestClosedLoopGoal() {
    return Math.abs(mLeaderFlywheelInputs.iFlywheelRotorVelocityRPS.getRotations() - mLastestClosedLoopGoalRPS.getRotations()) <= tFlywheelTolerance.get();
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothPDConstants(tFlywheelKP.get(), tFlywheelKD.get()), 
      tFlywheelKP, tFlywheelKD
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mLeaderFlywheelIO.setMotionMagicConstants(tFlywheelMaxVel.get(), tFlywheelMaxAccel.get(), tFlywheelMaxJerk.get()),
      tFlywheelMaxVel, tFlywheelMaxAccel, tFlywheelMaxJerk
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setFF(tFlywheelKS.get(), tFlywheelKV.get(), tFlywheelKA.get()),
      tFlywheelKS, tFlywheelKV, tFlywheelKA
    );
  }
}
