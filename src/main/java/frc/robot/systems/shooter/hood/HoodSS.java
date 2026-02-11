package frc.robot.systems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.shooter.ShooterConstants.HoodConstants;

public class HoodSS extends SubsystemBase{
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

  public HoodSS(HoodIO pHoodIO) {
    this.mHoodIO = pHoodIO;
    this.mHoodFF = HoodConstants.kHoodControlConfig.feedforward();
  }

  public void setHoodVolts(double pVolts) {
    mHoodIO.setMotorVolts(pVolts);
  }

  public void holdHood() {
    mHoodIO.setMotorPosition(mHoodInputs.iHoodAngle, mHoodInputs.iHoodVelocityRPS);
  }

  public void setHoodRot(Rotation2d pRotSP) {
    mHoodIO.setMotorPosition(pRotSP, mHoodFF.calculate(pRotSP.getRadians(), mHoodInputs.iHoodVelocityRPS));
  }

  public void stopHoodMotor() {
    mHoodIO.stopMotor();
  }

  private void setFF(double kS, double kG, double kV, double kA) {
    mHoodFF.setKs(kS);
    mHoodFF.setKg(kG);
    mHoodFF.setKv(kV);
    mHoodFF.setKa(kA);
  }
  
  @Override
  public void periodic() {
    mHoodIO.updateInputs(mHoodInputs);

    refreshTuneables();
    mHoodIO.stopIfCrazy();

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
