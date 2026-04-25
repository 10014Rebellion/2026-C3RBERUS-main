package frc.robot.systems.music;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MusicSS {
    private final MusicIODevice mIO;
    private final MusicInputsAutoLogged mInputs = new MusicInputsAutoLogged();

    public MusicSS(MusicIODevice pIO) {
        this.mIO = pIO;
    }

    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Music/", mInputs);
    }
}
