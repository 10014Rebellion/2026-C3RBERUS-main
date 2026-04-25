package frc.robot.systems.music;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.systems.music.MusicIO.MusicInputs;

public class MusicIODevice implements MusicIO {
    private final Orchestra mOrchestra;

    public MusicIODevice() {
        mOrchestra = new Orchestra();
    }

    @Override
    public void updateInputs(MusicInputs pInputs) {
        pInputs.iTimeStamp = mOrchestra.getCurrentTime();
        pInputs.iIsMusicPlaying = mOrchestra.isPlaying();
    }

    @Override
    public void addInstruments(TalonFX pDriveMotor, TalonFX pAzimuthMotor) {
        mOrchestra.addInstrument(pDriveMotor);
        mOrchestra.addInstrument(pAzimuthMotor);
    }

    @Override
    public void playOrchestra() {
        mOrchestra.play();
    }
}
