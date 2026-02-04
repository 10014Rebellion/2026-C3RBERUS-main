package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheels extends SubsystemBase {
  private final FlywheelIO mLeaderFlywheelIO;
  private final FlywheelIO mFollowerFlywheelIO;

  private final DCMotor mSimMotor;

  private final FlywheelInputsAutoLogged mLeaderFlywheelInputs = new FlywheelInputsAutoLogged();
  private final FlywheelInputsAutoLogged mFollowerFlywheelInputs = new FlywheelInputsAutoLogged();

  public Flywheels(FlywheelIO pLeaderFlywheelIO, FlywheelIO pFollowerFlywheelIO) {
    this.mLeaderFlywheelIO = pLeaderFlywheelIO;
    this.mFollowerFlywheelIO = pFollowerFlywheelIO;
    this.mSimMotor = DCMotor.getKrakenX44Foc(2);
  }

  public void setFlywheelVolts(double pVolts) {
    mLeaderFlywheelIO.setMotorVolts(pVolts);
    mFollowerFlywheelIO.enforceFollower();
  }

  public void setFlywheelSpeeds(double pRPS) {
    mLeaderFlywheelIO.setMotorVelAndAccel(pRPS, 0, mSimMotor.getCurrent(mLeaderFlywheelInputs.iFlywheelClosedLoopReference * Math.PI * 2, mLeaderFlywheelInputs.iFlywheelMotorVolts));
    mFollowerFlywheelIO.enforceFollower();
  }

  public void stopFlywheelMotor() {
    mLeaderFlywheelIO.stopMotor();
    mFollowerFlywheelIO.enforceFollower();
  }
  
  @Override
  public void periodic() {
    mLeaderFlywheelIO.updateInputs(mLeaderFlywheelInputs);
    mFollowerFlywheelIO.updateInputs(mFollowerFlywheelInputs);
    
    mFollowerFlywheelIO.enforceFollower(); // Sometimes during enable and disable it no longer follows briefly, this scares me, so this is a failsafe

    Logger.processInputs("Flywheel/Leader", mLeaderFlywheelInputs);
    Logger.processInputs("Flywheel/Follower", mFollowerFlywheelInputs);    
  }
}
