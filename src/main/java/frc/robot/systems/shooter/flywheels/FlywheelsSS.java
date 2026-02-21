package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLogOutput;
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

  private final LoggedTunableNumber tFlywheelTolerance = new LoggedTunableNumber("Flywheel/BangBang/Tolerance", ShooterConstants.FlywheelConstants.kTolerance);
  private final LoggedTunableNumber tFlywheelTimeout = new LoggedTunableNumber("Flywheel/BangBang/Timeout", ShooterConstants.FlywheelConstants.kBangBangTimeout);

  private double mTolerance = ShooterConstants.FlywheelConstants.kTolerance;
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

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/ErrorRotationsPerSec")
  public double getErrorRotationsPerSec() {
    return mCurrentRPS - getFlywheelRPS();
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/AtGoal")
  public boolean atGoal() {
    return Math.abs(getErrorRotationsPerSec()) < mTolerance;
  }
  
  @Override
  public void periodic() {
    mLeaderFlywheelIO.updateInputs(mLeaderFlywheelInputs);
    mFollowerFlywheelIO.updateInputs(mFollowerFlywheelInputs);
    mFlywheelEncoder.updateInputs(mEncoderInputs);
    
    refreshTuneables();
    mFollowerFlywheelIO.enforceFollower(); // Sometimes during enable and disable it no longer follows briefly, this scares me, so this is a failsafe

    double ffOutput = mFlywheelFeedforward.calculateWithVelocities(getFlywheelRPS(), mCurrentRPS);

    if(mCurrentState.equals(FlywheelState.TORQUE_FOC)){
      mLeaderFlywheelIO.setMotorVelAndAccel(mCurrentRPS, 0, ffOutput);
      mFollowerFlywheelIO.enforceFollower();
    }

    else{
      if(getFlywheelRPS() < mCurrentRPS){
        // Send max voltage until above setpoint //
        setFlywheelVolts(12.0);
      }

      boolean inTolerance = Math.abs(getFlywheelRPS() - mCurrentRPS) <= mTolerance;
      boolean atGoal = mAtGoalDebouncer.calculate(inTolerance);

        if(atGoal){
          mLeaderFlywheelIO.setMotorVelAndAccel(mCurrentRPS - 30, 0, ffOutput);
          mFollowerFlywheelIO.enforceFollower();
        }

        else{
          mLeaderFlywheelIO.setMotorVelAndAccel(mCurrentRPS + 10, 0, ffOutput);
          mFollowerFlywheelIO.enforceFollower();  
        }
    }

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
