package frc.robot.systems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.shooter.flywheels.Flywheels;
import frc.robot.systems.shooter.hood.Hood;
import frc.robot.systems.shooter.indexers.Indexers;

public class Shooter extends SubsystemBase {
  private final Indexers mIndexersSS;
  private final Hood mHoodSS;
  private final Flywheels mFlywheelSS;

  public Shooter(Indexers pIndexerSS, Hood pHoodSS, Flywheels pFlywheelSS) {
    this.mIndexersSS = pIndexerSS; this.mHoodSS = pHoodSS; this.mFlywheelSS = pFlywheelSS; // I miss my C++ initializer lists :'(
  }

  public Command setFlywheelsRPSCmd(double pRPS) {
    return new InstantCommand(() -> mFlywheelSS.setFlywheelSpeeds(pRPS));
  }

  public Command setFlywheelsVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mFlywheelSS.setFlywheelVolts(pVolts));
  }

  public Command setIndexersVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mIndexersSS.setIndexerVolts(pVolts));
  }

  public Command setHoodRot(Rotation2d pRot) {
    return new InstantCommand(() -> mHoodSS.setHoodRot(pRot));
  }

  public Command holdHoodCmd() {
    return new InstantCommand(() -> mHoodSS.holdHood());
  }

  public Command setHoodVoltsCmd(double pVolts) {
    return new InstantCommand(() -> mHoodSS.setHoodVolts(pVolts));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
