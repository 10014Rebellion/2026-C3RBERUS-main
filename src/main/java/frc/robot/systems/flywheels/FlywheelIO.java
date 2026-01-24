package frc.robot.systems.flywheels;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;


public interface FlywheelIO {
    @AutoLog
    public static class FlywheelInputs {
        /*TODO: logged everything except applied volts. should i add that?*/
        public double iFlywheelLeftVelocityMPS = 0.0;
        public double iFlywheelLeftStatorCurrentAmps = 0.0;
        public double iFlywheelLeftSupplyCurrentAmps = 0.0;
        public double iFlywheelLeftTorqueCurrentAmps = 0.0;
        public double iFlywheelLeftTemperatureCelsius = 0.0;
        public double iFlywheelLeftMotorVolts = 0.0;
        public double iFlywheelLeftAccelerationMPSS = 0.0;
        public Rotation2d iFlywheelLeftPosition = new Rotation2d();

        public double iFlywheelRightVelocityMPS = 0.0;
        public double iFlywheelRightStatorCurrentAmps = 0.0;
        public double iFlywheelRightSupplyCurrentAmps = 0.0;
        public double iFlywheelRightTorqueCurrentAmps = 0.0;
        public double iFlywheelRightTemperatureCelsius = 0.0;
        public double iFlywheelRightMotorVolts = 0.0;
        public double iFlywheelRightAccelerationMPSS = 0.0;
        public Rotation2d iFlywheelRightPosition = new Rotation2d();

    }

    public default void updateInputs(FlywheelInputs inputs) {}

    public default void setLeftFlywheelVolts(double volts) {}

    public default void setLeftFlywheelPID(double kP, double kI, double kD, double kV, double kA) {}

    public default void setLeftFlywheelVelocity(AngularVelocity setpointRPS, double pFF){}
    
    public default void setRightFlywheelVolts(double volts) {}

    public default void setRightFlywheelPID(double kP, double kI, double kD, double kV, double kA) {}

    public default void setRightFlywheelVelocity(AngularVelocity setpointRPS, double pFF){}

}
