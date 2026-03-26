package frc.robot.systems.shooter.fuelpump;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.logging.InvalidValueErrors.UnaccountedEnum;

import static frc.robot.systems.shooter.fuelpump.FuelPumpConstants.kFuelPumpControlConfig;

public class FuelPumpSS extends SubsystemBase {
  @AutoLogOutput(key = "Shooter/FuelPump/States/CurrentState")
  private String mCurrentFuelPumpBehavior = "STOPPED";

  private final FuelPumpIO mLeaderFuelPumpIO;
  private final FuelPumpIO mFollowerFuelPumpIO;

  private final FuelPumpInputsAutoLogged mLeaderFuelPumpInputs = new FuelPumpInputsAutoLogged();
  private final FuelPumpInputsAutoLogged mFollowerFuelPumpInputs = new FuelPumpInputsAutoLogged();

  private final LoggedTunableNumber tFuelPumpKP = new LoggedTunableNumber("Shooter/FuelPump/Control/PID/kP", kFuelPumpControlConfig.pdController().kP());
  private final LoggedTunableNumber tFuelPumpKD = new LoggedTunableNumber("Shooter/FuelPump/Control/PID/kD", kFuelPumpControlConfig.pdController().kD());
  private final LoggedTunableNumber tFuelPumpKS = new LoggedTunableNumber("Shooter/FuelPump/Control/FF/kS", kFuelPumpControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFuelPumpKV = new LoggedTunableNumber("Shooter/FuelPump/Control/FF/kV", kFuelPumpControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFuelPumpKA = new LoggedTunableNumber("Shooter/FuelPump/Control/FF/kA", kFuelPumpControlConfig.feedforward().getKa());
  // private final LoggedTunableNumber tFuelPumpTolerance = new LoggedTunableNumber("Shooter/FuelPump/Control/Tolerance", FuelPumpConstants.kToleranceRPS);

  private SimpleMotorFeedforward mFuelPumpFeedforward = new SimpleMotorFeedforward(tFuelPumpKS.get(), tFuelPumpKV.get(), tFuelPumpKA.get());

  public FuelPumpSS(FuelPumpIO pLeaderFuelPumpIO, FuelPumpIO pFollowerFuelPumpIO) {
    this.mLeaderFuelPumpIO = pLeaderFuelPumpIO;
    this.mFollowerFuelPumpIO = pFollowerFuelPumpIO;
  }

  @Override
  public void periodic() {
    mLeaderFuelPumpIO.updateInputs(mLeaderFuelPumpInputs);
    mFollowerFuelPumpIO.updateInputs(mFollowerFuelPumpInputs);

    refreshTuneables();
    executeState();
    mFollowerFuelPumpIO.enforceFollower();

    Logger.processInputs("Shooter/FuelPump/Leader", mLeaderFuelPumpInputs);
    Logger.processInputs("Shooter/FuelPump/Follower", mFollowerFuelPumpInputs);
  }

  private void executeState() {
    if(FuelPumpConstants.kStateToTuneableFuelPumpVelocity.containsKey(mCurrentFuelPumpBehavior)) {
      setFuelPumpVelocity(FuelPumpConstants.kStateToTuneableFuelPumpVelocity.get(mCurrentFuelPumpBehavior).get());
    } else if(FuelPumpConstants.kStateToTuneableFuelPumpVolts.containsKey(mCurrentFuelPumpBehavior)) {
      setFuelPumpVolts(FuelPumpConstants.kStateToTuneableFuelPumpVolts.get(mCurrentFuelPumpBehavior).get());
    } else {
      switch (mCurrentFuelPumpBehavior) {
        case "STOPPED" -> stopMotors();
        case "TUNING_VOLT" -> setFuelPumpVolts(FuelPumpConstants.tTuningVoltage.get());
        default -> {
          // unknown behavior string - no-op
        }
      }
    }
  }

  private void setFuelPumpVelocity(Rotation2d pRPS) {
    mLeaderFuelPumpIO.setMotorVelocity(
      pRPS,
      kFuelPumpControlConfig.feedforward().calculate(pRPS.getRotations())
    );
    mFollowerFuelPumpIO.enforceFollower();
  }

  private void setFuelPumpVolts(double pVoltage) {
    mLeaderFuelPumpIO.setMotorVolts(pVoltage);
    mFollowerFuelPumpIO.enforceFollower();
  }

  private void stopMotors() {
    mLeaderFuelPumpIO.stopMotor();
    mFollowerFuelPumpIO.enforceFollower();
  }

  public Command setStateCmd(String pNewBehavior) { return setStateCmd(pNewBehavior, true); }

  public Command setStateCmd(String pNewBehavior, boolean holdRequirementContinuously) {
    if (holdRequirementContinuously) {
      return Commands.startEnd(() -> setState(pNewBehavior), () -> {}, this);
    } else {
      return Commands.runOnce(() -> setState(pNewBehavior));
    }
  }

  private void setState(String pNewBehavior) {
    mCurrentFuelPumpBehavior = pNewBehavior;
  }

  public Rotation2d getAvgFuelPumpRPS() {
    return (mLeaderFuelPumpInputs.iFuelPumpVelocityRPS.plus(mFollowerFuelPumpInputs.iFuelPumpVelocityRPS)).div(2.0);
  }

  public boolean isReadyToShoot() {
    return getAvgFuelPumpRPS().getRotations() >= FuelPumpConstants.kRPSForShooting.getRotations();
  }

  public void setBothPDConstants(double KP, double KD){
    mFollowerFuelPumpIO.setPDConstants(0, KP, KD);
    mLeaderFuelPumpIO.setPDConstants(0, KP, KD);
  }

  public void setFF(double pKS, double pKV, double pKA){
    mFuelPumpFeedforward.setKs(pKS);
    mFuelPumpFeedforward.setKv(pKV);
    mFuelPumpFeedforward.setKa(pKA);
  }

  /* Convenience command factories for FuelPump states */
  public Command stoppedCmd() { return setStateCmd("STOPPED"); }
  public Command tuningVoltCmd() { return setStateCmd("TUNING_VOLT"); }
  public Command intakeVoltCmd() { return setStateCmd("INTAKE_VOLT"); }
  public Command intakeVelocityCmd() { return setStateCmd("INTAKE_VELOCITY"); }
  public Command outtakeVoltCmd() { return setStateCmd("OUTTAKE_VOLT"); }

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
