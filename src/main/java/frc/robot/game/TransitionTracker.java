package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.telemetry.Telemetry;

public class TransitionTracker {
  private Alliance mCurrentActiveAlliance;
  

  public TransitionTracker() {
  }

  public void periodic() {
  }

  public Alliance getFirstInactiveAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();
    if(gameData.length() > 0){
    switch (Character.toUpperCase(gameData.charAt(0))){
      case 'B':
        return Alliance.Blue;
      case 'R':
        return Alliance.Red;
      default: // Data is screwed.
        Telemetry.log("<<< DATA IS CORRUPTED >>>");
    }
    } else {
      Telemetry.log("<<< UNSURE WHICH TEAM IS INACTIVE FIRST >>>");
    }

    return null;
  }
}
