// REBELLION 10014

package frc.robot.systems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroInputs {
        public boolean iConnected = false;
        public Rotation2d iYawPosition = new Rotation2d();
        public Rotation2d iYawVelocityPS = Rotation2d.fromDegrees(0.0);
    }

    public default void updateInputs(GyroInputs inputs) {}

    public default void resetGyro(Rotation2d rotation) {}
}
