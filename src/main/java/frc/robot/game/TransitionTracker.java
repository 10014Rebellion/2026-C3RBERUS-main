package frc.robot.game;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.telemetry.Telemetry;

public class TransitionTracker {
  private double mAutonTimeLeft = -1;
  private double mTeleopTimeLeft = -1;
  private double mTimeLeftInPhase = -1;
  private GameState mCurrentState = GameState.STANDBY;
  private boolean mIsHubActive = true;
  private Alliance mOurAlliance = null;
  private Alliance mFirstActiveAlliance = null;

  public enum GameState {
    STANDBY, // Basically when robot is booted before match starts, if we're disabled mid match we wont be "standby", this timer is independent from the robot
    AUTON,
    TRANSITION,
    PHASE_1,
    PHASE_2,
    PHASE_3,
    PHASE_4,
    ENDGAME
  }

  public TransitionTracker() {
  }

  public void autonInit() {
    mOurAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    mAutonTimeLeft = Timer.getMatchTime();
    mCurrentState = GameState.AUTON;
  }

  public void teleopInit() {
    mOurAlliance = DriverStation.getAlliance().orElse(Alliance.Blue); // This is only needed if we're not running a match
    mAutonTimeLeft = -1;
    mTeleopTimeLeft = Timer.getMatchTime();
    mCurrentState = GameState.TRANSITION;
  }

  // Times derived from "https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf" page 36
  public void periodic() {
    double matchTimeLeft = Timer.getMatchTime();
    
    if(mCurrentState != GameState.STANDBY) {
      
      if (DriverStation.isAutonomous()) {
        mAutonTimeLeft = matchTimeLeft;
      } else {
        double teleopTimeElapsed = (2 * 60 + 20) - matchTimeLeft;

        if(teleopTimeElapsed < 10) { // Transition period
          
        } else if (teleopTimeElapsed < 35) { // Phase 1
          mTimeLeftInPhase = 35 - teleopTimeElapsed;
        } else if (teleopTimeElapsed < 60) { // Phase 2
          mTimeLeftInPhase = 60 - teleopTimeElapsed;
        } else if (teleopTimeElapsed < 85) { // Phase 3
          mTimeLeftInPhase = 85 - teleopTimeElapsed;
        } else if (teleopTimeElapsed < 110) { // Phase 4
          mTimeLeftInPhase = 110 - teleopTimeElapsed;
        } else { // Endgame
          mTimeLeftInPhase = -1;

        }
      }
    }
  }

  public Alliance getFirstActiveAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();
    if(gameData.length() > 0){
    switch (Character.toUpperCase(gameData.charAt(0))){
      // Blue and red are fipped since gameData returns the first INACTIVE alliance, thus the other side must be active
      case 'B':
        return Alliance.Red;
      case 'R':
        return Alliance.Blue;
      default: // Data is screwed.
        Telemetry.log("<<< DATA IS CORRUPTED >>>");
    }
    } else {
      Telemetry.log("<<< UNSURE WHICH TEAM IS INACTIVE FIRST >>>");
    }

    return null;
  }
}
