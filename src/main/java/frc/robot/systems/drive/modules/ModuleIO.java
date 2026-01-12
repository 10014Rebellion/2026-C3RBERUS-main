// REBELLION 10014

package frc.robot.systems.drive.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleInputs {
        public boolean iIsDriveConnected = true;
        public double iDrivePositionM = 0.0;
        public double iDriveVelocityMPS = 0.0;
        public double iDriveStatorCurrentAmps = 0.0;
        public double iDriveSupplyCurrentAmps = 0.0;
        public double iDriveTorqueCurrentAmps = 0.0;
        public double iDriveTemperatureCelsius = 0.0;
        public double iDriveAppliedVolts = 0.0;
        public double iDriveMotorVolts = 0.0;
        public double iDriveAccelerationMPSS = 0.0;

        public boolean iIsAzimuthConnected = true;
        public Rotation2d iAzimuthPosition = new Rotation2d();
        public Rotation2d iAzimuthVelocity = new Rotation2d();
        public double iAzimuthStatorCurrentAmps = 0.0;
        public double iAzimuthSupplyCurrentAmps = 0.0;

        public double iAzimuthTemperatureCelsius = 0.0;
        public double iAzimuthAppliedVolts = 0.0;
        public double iAzimuthMotorVolts = 0.0;

        public boolean iIsCancoderConnected = true;
        public Rotation2d iAzimuthAbsolutePosition = new Rotation2d();
    }

    public default void updateInputs(ModuleInputs inputs) {}

    public default void setDriveVelocity(double velocityMPS, double feedforward) {}

    public default void setDriveVolts(double volts) {}

    public default void setDriveAmperage(double amps) {}

    public default void setDrivePID(double kP, double kI, double kD) {}

    public default void setAzimuthVolts(double votls) {}

    public default void setAzimuthPosition(Rotation2d rotation, double feedforward) {}

    public default void resetAzimuthEncoder() {}

    public default void setAzimuthPID(double kP, double kI, double kD) {}
}
