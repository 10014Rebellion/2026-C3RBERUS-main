package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.shooter.flywheels.encoder.EncoderIO;
import frc.robot.systems.shooter.flywheels.encoder.EncoderInputsAutoLogged;
import static frc.robot.systems.shooter.ShooterConstants.FlywheelConstants.kFlywheelControlConfig;

public class FlywheelsSS extends SubsystemBase {
  private final FlywheelIO mLeaderFlywheelIO;
  private final FlywheelIO mFollowerFlywheelIO;
  private final EncoderIO mFlywheelEncoder;

  private final FlywheelInputsAutoLogged mLeaderFlywheelInputs = new FlywheelInputsAutoLogged();
  private final FlywheelInputsAutoLogged mFollowerFlywheelInputs = new FlywheelInputsAutoLogged();
  private final EncoderInputsAutoLogged mEncoderInputs = new EncoderInputsAutoLogged();

  private final LoggedTunableNumber tFlywheelCustomSetpointRPS = new LoggedTunableNumber("Flywheel/Control/CustomSetpointRPS", 0);

  private final LoggedTunableNumber tFlywheelKP = new LoggedTunableNumber("Flywheel/Control/kP", kFlywheelControlConfig.pdController().kP());
  private final LoggedTunableNumber tFlywheelKD = new LoggedTunableNumber("Flywheel/Control/kD", kFlywheelControlConfig.pdController().kD());

  private final LoggedTunableNumber tFlywheelKS = new LoggedTunableNumber("Flywheel/Control/kS", kFlywheelControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFlywheelKV = new LoggedTunableNumber("Flywheel/Control/kV", kFlywheelControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFlywheelKA = new LoggedTunableNumber("Flywheel/Control/kA", kFlywheelControlConfig.feedforward().getKa());

  private final LoggedTunableNumber tFlywheelCruiseVel = new LoggedTunableNumber("Flywheel/Control/CruiseVel", kFlywheelControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tFlywheelMaxAccel = new LoggedTunableNumber("Flywheel/Control/MaxAcceleration", kFlywheelControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tFlywheelMaxJerk = new LoggedTunableNumber("Flywheel/Control/MaxJerk", kFlywheelControlConfig.motionMagicConstants().maxJerk());

  public FlywheelsSS(FlywheelIO pLeaderFlywheelIO, FlywheelIO pFollowerFlywheelIO, EncoderIO pFlywheelEncoder) {
    this.mLeaderFlywheelIO = pLeaderFlywheelIO;
    this.mFollowerFlywheelIO = pFollowerFlywheelIO;
    this.mFlywheelEncoder = pFlywheelEncoder;
  }

  public void setFlywheelVolts(double pVolts) {
    mLeaderFlywheelIO.setMotorVolts(pVolts);
    mFollowerFlywheelIO.enforceFollower();
  }

  public void setFlywheelSpeeds(Rotation2d pRotPerS) {
    mLeaderFlywheelIO.setMotorVelAndAccel(pRotPerS.getRotations(), 0, kFlywheelControlConfig.feedforward().calculate(pRotPerS.getRotations()));
    mFollowerFlywheelIO.enforceFollower();
  }

  public void setFlywheelSpeeds() {
    setFlywheelSpeeds(Rotation2d.fromRotations(tFlywheelCustomSetpointRPS.getAsDouble()));
  }

  public double getFlywheelRPS() {
    return mEncoderInputs.iEncoderVelocityRPS.getRotations();
  }

  public void stopFlywheelMotor() {
    mLeaderFlywheelIO.stopMotor();
    mFollowerFlywheelIO.enforceFollower();
  }

  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFlywheelIO.setPDConstants(pKP, pKD);
    mFollowerFlywheelIO.setPDConstants(pKP, pKD);
  }

  private void setBothMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
    mLeaderFlywheelIO.setMotionMagicConstants(pCruiseVel, pMaxAccel, pMaxJerk);
    mFollowerFlywheelIO.setMotionMagicConstants(pCruiseVel, pMaxAccel, pMaxJerk);
  }

  private void setFF(double pKS, double pKV, double pKA){
    kFlywheelControlConfig.feedforward().setKv(pKV);
    kFlywheelControlConfig.feedforward().setKs(pKS);
    kFlywheelControlConfig.feedforward().setKa(pKA);
  }
  
  @Override
  public void periodic() {
    mLeaderFlywheelIO.updateInputs(mLeaderFlywheelInputs);
    mFollowerFlywheelIO.updateInputs(mFollowerFlywheelInputs);
    mFlywheelEncoder.updateInputs(mEncoderInputs);
    
    refreshTuneables();
    
    // Help conserve some power on the motor), since its a 1 way bearing
    if(mLeaderFlywheelInputs.iFlywheelMotorVolts < 0){
      setFlywheelVolts(0.0);
    } else {
      mFollowerFlywheelIO.enforceFollower(); // Sometimes during enable and disable it no longer follows briefly, this scares me, so this is a failsafe
    }

    Logger.processInputs("Flywheel/Leader", mLeaderFlywheelInputs);
    Logger.processInputs("Flywheel/Follower", mFollowerFlywheelInputs);
    Logger.processInputs("Flywheel/Encoder", mEncoderInputs);
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
}
