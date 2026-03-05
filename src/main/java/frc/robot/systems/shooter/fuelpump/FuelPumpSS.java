package frc.robot.systems.shooter.fuelpump;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.shooter.ShooterConstants.FuelPumpConstants;

public class FuelPumpSS extends SubsystemBase {

  public static enum FuelPumpState {
    STOPPED(() -> 0.0),
    INTAKE(() -> 11.0),
    UNJAM(() -> -2),
    OUTTAKE(() -> -10.014),
    SLOW_OUTTAKE(() -> -4),
    TUNING(null);

    private DoubleSupplier mVoltage;

    private FuelPumpState(DoubleSupplier pVoltage) {
      mVoltage = pVoltage;
    }

    public double getDesiredVoltge() {
      return mVoltage.getAsDouble();
    } 
  }


  private final FuelPumpIO mLeaderFuelPumpIO;
  private final FuelPumpIO mFollowerFuelPumpIO;

  private final FuelPumpInputsAutoLogged mLeaderFuelPumpInputs = new FuelPumpInputsAutoLogged();
  private final FuelPumpInputsAutoLogged mFollowerFuelPumpInputs = new FuelPumpInputsAutoLogged();

  private FuelPumpState mFuelPumpState = FuelPumpState.STOPPED;
  
  public FuelPumpSS(FuelPumpIO pLeaderFuelPumpIO, FuelPumpIO pFollowerFuelPumpIO) {
    this.mLeaderFuelPumpIO = pLeaderFuelPumpIO;
    this.mFollowerFuelPumpIO = pFollowerFuelPumpIO;
  }
  
  @Override
  public void periodic() {
    mLeaderFuelPumpIO.updateInputs(mLeaderFuelPumpInputs);
    mFollowerFuelPumpIO.updateInputs(mFollowerFuelPumpInputs);

    mFollowerFuelPumpIO.enforceFollower();

    if(mFuelPumpState != null) Logger.recordOutput("FuelPump/State", mFuelPumpState);

    Logger.processInputs("FuelPump/Leader", mLeaderFuelPumpInputs);
    Logger.processInputs("FuelPump/Follower", mFollowerFuelPumpInputs);
  }

  public Command setFuelPumpStateCmd(FuelPumpState pFuelPumpState) {
    return Commands.run(() -> {
      mFuelPumpState = pFuelPumpState;
      mLeaderFuelPumpIO.setMotorVolts(pFuelPumpState.getDesiredVoltge());
    }, this);
  }

  public Command setFuelPumpManualCmd(double pVolts) {
    return Commands.run(() -> {
      mLeaderFuelPumpIO.setMotorVolts(pVolts);
    }, this);
  }

  public Command stopFuelPumpCmd() {
    return Commands.run(() -> {
      stopFuelPumpMotors();
    }, this);
  }

  public Rotation2d getAvgFuelPumpRPS() {
    return (mLeaderFuelPumpInputs.iFuelPumpVelocityRPS.plus(mFollowerFuelPumpInputs.iFuelPumpVelocityRPS)).div(2.0);
  }

  public boolean isReadyToShoot() {
    return getAvgFuelPumpRPS().getRotations() >= FuelPumpConstants.kRPSForShooting.getRotations();
  }

  public void setFuelPumpVolts(double pVolts) {
    mLeaderFuelPumpIO.setMotorVolts(pVolts);
    mFollowerFuelPumpIO.enforceFollower();
  }

  public void setFuelPumpVoltsManual(double pVolts) {
    mFuelPumpState = null;
    mLeaderFuelPumpIO.setMotorVolts(pVolts);
    mFollowerFuelPumpIO.enforceFollower();
  }
  
  public void stopFuelPumpMotors() {
    mFuelPumpState = null;
    mLeaderFuelPumpIO.stopMotor();
    mFollowerFuelPumpIO.enforceFollower();
  }
  
  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFuelPumpIO.setPDConstants(0, pKP, pKD);
    mFollowerFuelPumpIO.setPDConstants(0, pKP, pKD);
  }
}
