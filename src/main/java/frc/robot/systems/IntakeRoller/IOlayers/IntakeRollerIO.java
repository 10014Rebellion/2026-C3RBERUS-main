package frc.robot.systems.IntakeRoller.IOlayers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
    
    @AutoLog
    public static class IntakeRollerIOInputs {
        public double innerRollerVoltage = 0.0;
        public double innerRollerCurrent = 0.0;
        public double innerRollerVelocity = 0.0;
        public double innerRollerTemperature = 0.0;
        
        public double outerRollerVoltage = 0.0;
        public double outerRollerCurrent = 0.0;
        public double outerRollerVelocity = 0.0;
        public double outerRollerTemperature = 0.0;
        
        public boolean fuelDetected = false;
    }
    
    public default void updateInputs(IntakeRollerIOInputs inputs) {}
    
    public default void setInnerVoltage(double volts) {}
    
    public default void setOuterVoltage(double volts) {}
    
    public default void stop() {}
}