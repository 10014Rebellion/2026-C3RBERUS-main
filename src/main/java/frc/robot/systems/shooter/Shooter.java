package frc.robot.systems.shooter;

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
