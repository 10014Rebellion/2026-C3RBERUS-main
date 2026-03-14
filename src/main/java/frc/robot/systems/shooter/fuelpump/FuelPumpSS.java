package frc.robot.systems.shooter.fuelpump;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.robot.logging.InvalidValueErrors.UnaccountedEnum;

public class FuelPumpSS extends SubsystemBase {
  public static enum FuelPumpState {
    STOPPED,
    TUNING_VOLT,
    INTAKE_VOLT,
    OUTTAKE_VOLT,
  }

  @AutoLogOutput(key = "Shooter/FuelPump/States/CurrentState")
  private FuelPumpState mCurrentFuelPumpState = FuelPumpState.STOPPED;

  private final FuelPumpIO mLeaderFuelPumpIO;
  private final FuelPumpIO mFollowerFuelPumpIO;

  private final FuelPumpInputsAutoLogged mLeaderFuelPumpInputs = new FuelPumpInputsAutoLogged();
  private final FuelPumpInputsAutoLogged mFollowerFuelPumpInputs = new FuelPumpInputsAutoLogged();

  public FuelPumpSS(FuelPumpIO pLeaderFuelPumpIO, FuelPumpIO pFollowerFuelPumpIO) {
    this.mLeaderFuelPumpIO = pLeaderFuelPumpIO;
    this.mFollowerFuelPumpIO = pFollowerFuelPumpIO;
  }

  @Override
  public void periodic() {
    mLeaderFuelPumpIO.updateInputs(mLeaderFuelPumpInputs);
    mFollowerFuelPumpIO.updateInputs(mFollowerFuelPumpInputs);

    executeState();
    mFollowerFuelPumpIO.enforceFollower();

    Logger.processInputs("Shooter/FuelPump/Leader", mLeaderFuelPumpInputs);
    Logger.processInputs("Shooter/FuelPump/Follower", mFollowerFuelPumpInputs);
  }

  private void executeState() {
    if(FuelPumpConstants.kStateToTuneableFuelPump.containsKey(mCurrentFuelPumpState)) {
      setFuelPumpVolts(FuelPumpConstants.kStateToTuneableFuelPump.get(mCurrentFuelPumpState).get());
    } else {
      switch (mCurrentFuelPumpState) {
        case STOPPED -> {
          stopMotors();
        }
        case TUNING_VOLT -> {
          setFuelPumpVolts(FuelPumpConstants.tTuningVoltage.get());
        }
        default -> {
          Telemetry.reportIssue(new UnaccountedEnum(mCurrentFuelPumpState.toString()));
        }
      }
    }
  }

  private void setFuelPumpVolts(double pVoltage) {
    mLeaderFuelPumpIO.setMotorVolts(pVoltage);
    mFollowerFuelPumpIO.enforceFollower();
  }

  private void stopMotors() {
    mLeaderFuelPumpIO.stopMotor();
    mFollowerFuelPumpIO.enforceFollower();
  }

  public Command setStateCmd(FuelPumpState pNewState) {
    return setStateCmd(pNewState, true);
  }

  public Command setStateCmd(FuelPumpState pNewState, boolean holdRequirementContinuously) {
    return new FunctionalCommand(
      () -> setState(pNewState), () -> {}, (interrupted) -> {},
      () -> !holdRequirementContinuously, this);
  }

  private void setState(FuelPumpState pNewState) {
    mCurrentFuelPumpState = pNewState;
  }

  public Rotation2d getAvgFuelPumpRPS() {
    return (mLeaderFuelPumpInputs.iFuelPumpVelocityRPS.plus(mFollowerFuelPumpInputs.iFuelPumpVelocityRPS)).div(2.0);
  }

  public boolean isReadyToShoot() {
    return getAvgFuelPumpRPS().getRotations() >= FuelPumpConstants.kRPSForShooting.getRotations();
  }
}
