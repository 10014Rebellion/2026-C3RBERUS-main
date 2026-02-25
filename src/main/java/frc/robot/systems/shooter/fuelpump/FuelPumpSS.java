package frc.robot.systems.shooter.fuelpump;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.ShooterConstants;
import frc.robot.systems.shooter.ShooterConstants.FuelPumpConstants;

public class FuelPumpSS extends SubsystemBase {

    public static enum FuelPumpState {
    IDLE(() -> 0.0),
    INTAKE(() -> 10.014),
    OUTTAKE(() -> -10.014),
    TUNING(new LoggedTunableNumber("FuelPump/TuneableVoltage", 0.0));

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

  private final LoggedTunableNumber tFuelPumpKP = new LoggedTunableNumber("FuelPump/Control/kP", FuelPumpConstants.kFuelPumpControlConfig.pdController().kP());
  private final LoggedTunableNumber tFuelPumpKD = new LoggedTunableNumber("FuelPump/Control/kD", FuelPumpConstants.kFuelPumpControlConfig.pdController().kD());

  private final LoggedTunableNumber tFuelPumpKS = new LoggedTunableNumber("FuelPump/Control/kS", FuelPumpConstants.kFuelPumpControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFuelPumpKV = new LoggedTunableNumber("FuelPump/Control/kV", FuelPumpConstants.kFuelPumpControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFuelPumpKA = new LoggedTunableNumber("FuelPump/Control/kA", FuelPumpConstants.kFuelPumpControlConfig.feedforward().getKa());
  
  private final LoggedTunableNumber tFuelPumpTolerance = new LoggedTunableNumber("FuelPump/Control/Tolerance", ShooterConstants.FuelPumpConstants.kToleranceRPS);
  
  private SimpleMotorFeedforward mFuelPumpFeedForward = FuelPumpConstants.kFuelPumpControlConfig.feedforward();
  
  private Rotation2d mCurrentRPSGoal = Rotation2d.kZero;
  private FuelPumpState mFuelPumpState = FuelPumpState.IDLE;
  
  public FuelPumpSS(FuelPumpIO pLeaderFuelPumpIO, FuelPumpIO pFollowerFuelPumpIO) {
    this.mLeaderFuelPumpIO = pLeaderFuelPumpIO;
    this.mFollowerFuelPumpIO = pFollowerFuelPumpIO;
  }
  
  @Override
  public void periodic() {
    mLeaderFuelPumpIO.updateInputs(mLeaderFuelPumpInputs);
    mFollowerFuelPumpIO.updateInputs(mFollowerFuelPumpInputs);

    refreshTuneables();
    mFollowerFuelPumpIO.enforceFollower();

    Logger.processInputs("FuelPump/Leader", mLeaderFuelPumpInputs);
    Logger.processInputs("FuelPump/Follower", mFollowerFuelPumpInputs);

    if(mFuelPumpState != null) {
      Logger.recordOutput("FuelPump/DesiredVoltage", mFuelPumpState.getDesiredVoltge());

      setFuelPumpVolts(mFuelPumpState.getDesiredVoltge());
    }
  }

  public Command setFuelPumpStateCmd(FuelPumpState pFuelPumpState) {
    return Commands.run(() -> {
      mFuelPumpState = pFuelPumpState;
    }, this);
  }

  public Command setFuelPumpManualCmd(double pVolts) {
    return Commands.run(() -> {
      mFuelPumpState = null;
      mLeaderFuelPumpIO.setMotorVolts(pVolts);
    }, this);
  }

  public Command stopFuelPumpCmd() {
    return Commands.run(() -> {
      mFuelPumpState = null;
      stopFuelPumpMotors();
    }, this);
  }

  public double getAvgFuelPumpRPS() {
    return (mLeaderFuelPumpInputs.iFuelPumpVelocityRPS + mFollowerFuelPumpInputs.iFuelPumpVelocityRPS) / 2.0;
  }
  
  public void setFuelPumpRPS(double pDesiredRPS) {
    mCurrentRPSGoal = Rotation2d.fromRotations(pDesiredRPS);
    double calculatedFF = mFuelPumpFeedForward.calculateWithVelocities(getAvgFuelPumpRPS(), pDesiredRPS);
    mLeaderFuelPumpIO.setMotorVelocity(pDesiredRPS, calculatedFF);
    mFollowerFuelPumpIO.enforceFollower();
  }

  public void setFuelPumpVolts(double pVolts) {
    mLeaderFuelPumpIO.setMotorVolts(pVolts);
    mFollowerFuelPumpIO.enforceFollower();
  }
  
  public void stopFuelPumpMotors() {
    mLeaderFuelPumpIO.stopMotor();
    mFollowerFuelPumpIO.enforceFollower();
  }
  
  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFuelPumpIO.setPDConstants(0, pKP, pKD);
    mFollowerFuelPumpIO.setPDConstants(0, pKP, pKD);
  }

  private void setFF(double pKS, double pKV, double pKA){
    mFuelPumpFeedForward.setKs(pKS);
    mFuelPumpFeedForward.setKv(pKV);
    mFuelPumpFeedForward.setKa(pKA);
  }

  @AutoLogOutput(key = "Shooter/FuelPump/Feedback/ErrorRotationsPerSec")
  public double getErrorRotationsPerSec() {
    return mCurrentRPSGoal.getRotations() - getAvgFuelPumpRPS();
  }

  @AutoLogOutput(key = "Shooter/FuelPump/Feedback/CurrentGoal")
  public Rotation2d getCurrentGoal() {
    return mCurrentRPSGoal;
  }

  @AutoLogOutput(key = "Shooter/FuelPump/Feedback/AtGoal")
  public boolean atGoal() {
    return Math.abs(getErrorRotationsPerSec()) < tFuelPumpTolerance.get();
  }


  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothPDConstants(tFuelPumpKP.get(), tFuelPumpKD.get()), 
      tFuelPumpKP, tFuelPumpKD
    );
  
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setFF(tFuelPumpKS.get(), tFuelPumpKV.get(), tFuelPumpKA.get()), 
      tFuelPumpKS, tFuelPumpKV, tFuelPumpKA
    );
  }
}
