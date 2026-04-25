package frc.robot.systems.music;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface MusicIO {

    @AutoLog
    public static class MusicInputs {
        public boolean iIsMusicPlaying = false;
        public double iTimeStamp = 0.0;
    }

    public default void updateInputs(MusicInputs pInputs) {}

    public default void addInstruments(TalonFX pDriveMotor, TalonFX pAzimuthMotor) {}

    public default void playOrchestra() {}
}
