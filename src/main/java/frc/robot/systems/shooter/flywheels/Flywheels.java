package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheels extends SubsystemBase {
  private final FlywheelIO mLeaderFlywheelIO;
  private final FlywheelIO mFollowerFlywheelIO;

  private final FlywheelInputsAutoLogged mLeaderFlywheelInputs = new FlywheelInputsAutoLogged();
  private final FlywheelInputsAutoLogged mFollowerFlywheelInputs = new FlywheelInputsAutoLogged();

  public Flywheels(FlywheelIO pLeaderFlywheelIO, FlywheelIO pFollowerFlywheelIO) {
    this.mLeaderFlywheelIO = pLeaderFlywheelIO;
    this.mFollowerFlywheelIO = pFollowerFlywheelIO;
  }

  public void setFlywheelVolts(double pVolts) {
    mLeaderFlywheelIO.setMotorVolts(pVolts);
  }

  public void stopFlywheelMotor() {
    mLeaderFlywheelIO.stopMotor();
  }
  
  @Override
  public void periodic() {
    mLeaderFlywheelIO.updateInputs(mLeaderFlywheelInputs);
    mFollowerFlywheelIO.updateInputs(mFollowerFlywheelInputs);
    Logger.processInputs("Flywheel/Leader", mLeaderFlywheelInputs);
    Logger.processInputs("Flywheel/Follower", mFollowerFlywheelInputs);
  }
}
