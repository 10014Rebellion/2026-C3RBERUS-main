package frc.robot.systems.flywheels;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;


public interface FlywheelIO {

    @AutoLog
    public static class FlywheelInputs {

    /*TODO: update the values I don't initilize later! */
    /*Let me know if I should add more for logging. Just base values for now */
    public double iFlywheelLeftVelocityMPS = 0.0;
    public double iFlywheelLeftStatorCurrentAmps = 0.0;
    public double iFlywheelLeftSupplyCurrentAmps = 0.0;
    public double iFlywheelLeftTorqueCurrentAmps = 0.0;
    public double iFlywheelLeftTemperatureCelsius = 0.0;
    public double iFlywheelLeftAppliedVolts = 0.0;
    public double iFlywheelLeftMotorVolts = 0.0;
    public double iFlywheelLeftAccelerationMPSS = 0.0;

    public double iFlywheelRightVelocityMPS = 0.0;
    public double iFlywheelRightStatorCurrentAmps = 0.0;
    public double iFlywheelRightSupplyCurrentAmps = 0.0;
    public double iFlywheelRightTorqueCurrentAmps = 0.0;
    public double iFlywheelRightTemperatureCelsius = 0.0;
    public double iFlywheelRightAppliedVolts = 0.0;
    public double iFlywheelRightMotorVolts = 0.0;
    public double iFlywheelRightAccelerationMPSS = 0.0;

    }

    /*TODO: DO I need more methods? */
    public default void updateInputs(FlywheelInputs inputs) {}

    public default void setLeftFlywheelVolts(double volts) {}

    public default void setLeftFlywheelPID(AngularVelocity setpointRPS) {}

    public default void setRightFlywheelVolts(double volts) {}

    public default void setRightFlywheelPID(AngularVelocity setpointRPS) {}

}
