package frc.robot.bindings;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;

public interface DigitalInputButtonIO {
    @AutoLog
    public static class DigitalInputButtonIOInputs {
        public boolean iInput = false;
    }

    public default void updateInputs(DigitalInputButtonIOInputs pInputs) {}
}
