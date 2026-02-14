package frc.robot.systems.shooter.fuelpump;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.shooter.ShooterConstants.FuelPumpConstants;

public class FuelPumpSS extends SubsystemBase {
  private final FuelPumpIO mLeaderFuelPumpIO;
  private final FuelPumpIO mFollowerFuelPumpIO;

  private final FuelPumpInputsAutoLogged mLeaderFuelPumpInputs = new FuelPumpInputsAutoLogged();
  private final FuelPumpInputsAutoLogged mFollowerFuelPumpInputs = new FuelPumpInputsAutoLogged();

  private final LoggedTunableNumber tFuelPumpKP = new LoggedTunableNumber("FuelPump/Control/kP", FuelPumpConstants.kFuelPumpControlConfig.pdController().kP());
  private final LoggedTunableNumber tFuelPumpKD = new LoggedTunableNumber("FuelPump/Control/kD", FuelPumpConstants.kFuelPumpControlConfig.pdController().kD());

  private final LoggedTunableNumber tFuelPumpKS = new LoggedTunableNumber("FuelPump/Control/kS", FuelPumpConstants.kFuelPumpControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFuelPumpKV = new LoggedTunableNumber("FuelPump/Control/kV", FuelPumpConstants.kFuelPumpControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFuelPumpKA = new LoggedTunableNumber("FuelPump/Control/kA", FuelPumpConstants.kFuelPumpControlConfig.feedforward().getKa());

  private final LoggedTunableNumber tFuelCruiseVel = new LoggedTunableNumber("Flywheel/Control/CruiseVel", FuelPumpConstants.kFuelPumpControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tFuelMaxAccel = new LoggedTunableNumber("Flywheel/Control/MaxAcceleration", FuelPumpConstants.kFuelPumpControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tFuelMaxJerk = new LoggedTunableNumber("Flywheel/Control/MaxJerk", FuelPumpConstants.kFuelPumpControlConfig.motionMagicConstants().maxJerk());
  
  private SimpleMotorFeedforward mFuelPumpFeedForward = FuelPumpConstants.kFuelPumpControlConfig.feedforward();
  
  public FuelPumpSS(FuelPumpIO pLeaderFuelPumpIO, FuelPumpIO pFollowerFuelPumpIO) {
    this.mLeaderFuelPumpIO = pLeaderFuelPumpIO;
    this.mFollowerFuelPumpIO = pFollowerFuelPumpIO;
  }

  public double getAvgFuelPumpRPS() {
    return (mLeaderFuelPumpInputs.iFuelPumpVelocityRPS + mFollowerFuelPumpInputs.iFuelPumpVelocityRPS) / 2.0;
  }

  public void setFuelPumpRPS(double pDesiredRPS) {
    double calculatedFF = mFuelPumpFeedForward.calculateWithVelocities(getAvgFuelPumpRPS(), pDesiredRPS);
    mLeaderFuelPumpIO.setMotorVelocity(pDesiredRPS, calculatedFF);
    mFollowerFuelPumpIO.enforceFollower();
  }

  public void setFuelPumpVolts(double pVolts) {
    mLeaderFuelPumpIO.setMotorVolts(pVolts);
    mFollowerFuelPumpIO.enforceFollower();
  }

  
  public void stopFuelPumpMotor() {
    mLeaderFuelPumpIO.stopMotor();
    mFollowerFuelPumpIO.enforceFollower();
  }
  
  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFuelPumpIO.setPDConstants(0, pKP, pKD);
    mFollowerFuelPumpIO.setPDConstants(0, pKP, pKD);
  }
  
  private void setBothMagicMotionConstraints(double pCruiseVelo, double pMaxAccel, double pMaxJerk){
    mLeaderFuelPumpIO.setMotionMagicConstants(pCruiseVelo, pMaxAccel, pMaxJerk);
    mFollowerFuelPumpIO.setMotionMagicConstants(pCruiseVelo, pMaxAccel, pMaxJerk);
  }
  
  @Override
  public void periodic() {
    mLeaderFuelPumpIO.updateInputs(mLeaderFuelPumpInputs);
    mFollowerFuelPumpIO.updateInputs(mFollowerFuelPumpInputs);

    refreshTuneables();
    mFollowerFuelPumpIO.enforceFollower();

    Logger.processInputs("FuelPump/Leader", mLeaderFuelPumpInputs);
    Logger.processInputs("FuelPump/Follower", mFollowerFuelPumpInputs);
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothPDConstants(tFuelPumpKP.get(), tFuelPumpKD.get()), 
      tFuelPumpKP, tFuelPumpKD
    );
  
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mFuelPumpFeedForward = new SimpleMotorFeedforward(tFuelPumpKS.get(), tFuelPumpKV.get(), tFuelPumpKA.get()), 
      tFuelPumpKS, tFuelPumpKV, tFuelPumpKA
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
    () -> setBothMagicMotionConstraints(tFuelCruiseVel.get(), tFuelMaxAccel.get(), tFuelMaxJerk.get()), 
    tFuelCruiseVel, tFuelMaxAccel, tFuelMaxJerk
    );
  }
}
