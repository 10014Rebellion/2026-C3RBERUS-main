package frc.robot.systems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AngularServoIO {
    @AutoLog
    public static class ServoIOInputs {
        boolean iUsingServo = false;
        double iServoPosition = 0.0;
        double iServoSpeed = 0.0;
        double iServoBoundMin = 0.0;
        double iServoBoundMax = 0.0;
        double iServoTime = 0.0;
        double iServoHandle = 0.0;
        double iServoChannel = 0.0;
    }

    public default void updateInputs(ServoIOInputs pInputs) {}

    public default void setPosition(Rotation2d pos) {}
}
