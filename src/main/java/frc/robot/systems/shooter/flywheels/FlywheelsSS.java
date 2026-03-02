package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.shooter.ShooterConstants;
import frc.robot.systems.shooter.ShooterConstants.FlywheelConstants.FlywheelState;
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

  private final LoggedTunableNumber tFlywheelKP = new LoggedTunableNumber("Flywheel/Control/kP", kFlywheelControlConfig.pdController().kP());
  private final LoggedTunableNumber tFlywheelKD = new LoggedTunableNumber("Flywheel/Control/kD", kFlywheelControlConfig.pdController().kD());

  private final LoggedTunableNumber tFlywheelKS = new LoggedTunableNumber("Flywheel/Control/kS", kFlywheelControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFlywheelKV = new LoggedTunableNumber("Flywheel/Control/kV", kFlywheelControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFlywheelKA = new LoggedTunableNumber("Flywheel/Control/kA", kFlywheelControlConfig.feedforward().getKa());

  private final LoggedTunableNumber tFlywheelCruiseVel = new LoggedTunableNumber("Flywheel/Control/CruiseVel", kFlywheelControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tFlywheelMaxAccel = new LoggedTunableNumber("Flywheel/Control/MaxAcceleration", kFlywheelControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tFlywheelMaxJerk = new LoggedTunableNumber("Flywheel/Control/MaxJerk", kFlywheelControlConfig.motionMagicConstants().maxJerk());

  private final LoggedTunableNumber tFlywheelTolerance = new LoggedTunableNumber("Flywheel/BangBang/Tolerance", ShooterConstants.FlywheelConstants.kBangBangTolerance);
  private final LoggedTunableNumber tFlywheelTimeout = new LoggedTunableNumber("Flywheel/BangBang/Timeout", ShooterConstants.FlywheelConstants.kBangBangTimeout);
  private double mTolerance = ShooterConstants.FlywheelConstants.kBangBangTolerance;
  private double mTimeout = ShooterConstants.FlywheelConstants.kBangBangTimeout;

  private SimpleMotorFeedforward mFlywheelFeedforward = kFlywheelControlConfig.feedforward();

  private Debouncer mAtGoalDebouncer = new Debouncer(mTimeout, DebounceType.kRising);

  private FlywheelState mCurrentState = FlywheelState.TORQUE_FOC;
  private double mCurrentRPS = 0.0;

  public FlywheelsSS(FlywheelIO pLeaderFlywheelIO, FlywheelIO pFollowerFlywheelIO, EncoderIO pFlywheelEncoder) {
    this.mLeaderFlywheelIO = pLeaderFlywheelIO;
    this.mFollowerFlywheelIO = pFollowerFlywheelIO;
    this.mFlywheelEncoder = pFlywheelEncoder;
  }

  public void setFlywheelVolts(double pVolts) {
    mLeaderFlywheelIO.setMotorVolts(pVolts);
    mFollowerFlywheelIO.enforceFollower();
  }

  public double getFlywheelRPS() {
    return mEncoderInputs.iEncoderVelocityRPS.getRotations();
  }

  public void setFlywheelSpeeds(FlywheelState pState, double pRPS){
    mCurrentState = pState;
    mCurrentRPS = pRPS;
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
    mFlywheelFeedforward.setKv(pKV);
    mFlywheelFeedforward.setKs(pKS);
    mFlywheelFeedforward.setKa(pKA);
  }
  
  @Override
  public void periodic() {
    mLeaderFlywheelIO.updateInputs(mLeaderFlywheelInputs);
    mFollowerFlywheelIO.updateInputs(mFollowerFlywheelInputs);
    mFlywheelEncoder.updateInputs(mEncoderInputs);
    
    refreshTuneables();
    mFollowerFlywheelIO.enforceFollower(); // Sometimes during enable and disable it no longer follows briefly, this scares me, so this is a failsafe

    // Help conserve some power on the motor)
    if(mLeaderFlywheelInputs.iFlywheelMotorVolts < 0){
      setFlywheelVolts(0.0);
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

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mTolerance = tFlywheelTolerance.get(),
      tFlywheelTolerance
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mAtGoalDebouncer.setDebounceTime(tFlywheelTimeout.get()),
      tFlywheelTimeout
    );
  }
}
