package frc.robot.systems.music;

import org.littletonrobotics.junction.AutoLog;

public class MusicIO {
    @AutoLog
    public static class ModuleInputs {
        public boolean iIsMusicPlaying = false;
        public double iTimeStamp = 0.0;
    }
}
