package frc.robot.systems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.indexers.IndexersSS;

public class Shooter {
  private final IndexersSS mIndexersSS;
  private final HoodSS mHoodSS;
  private final FlywheelsSS mFlywheelSS;

  public Shooter(IndexersSS pIndexerSS, HoodSS pHoodSS, FlywheelsSS pFlywheelSS) {
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

  public Command setIndexerRPSCmd(double pRPS) {
    return new InstantCommand(() -> mIndexersSS.setIndexerRPS(pRPS));
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
}
