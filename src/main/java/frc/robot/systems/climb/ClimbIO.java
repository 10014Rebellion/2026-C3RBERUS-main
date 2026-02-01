// REBELLION 10014

package frc.robot.systems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbInputs {
      public boolean iIsClimbConnected = false;
      public double iClimbVelocityMPS = 0.0;
      public double iClimbAccelerationMPSS = 0.0;
      public double iClimbMotorVolts = 0.0;
      public double iClimbSupplyCurrentAmps = 0.0;
      public double iClimbStatorCurrentAmps = 0.0;
      public double iClimbTempCelsius = 0.0;
      public double iClimbPositionMeters = 0.0;
    }

    public default void updateInputs(ClimbInputs pInputs) {}
    public default void setMotorVolts(double pVolts) {}
    public default void setMotorPosition(int pSlot, double pPositionM, double pFeedforward) {}
    public default void setPIDConstants(int pSlot, double kP, double kI, double kD) {};
    public default void setFFConstants(int pSlot, double kS, double kG, double kV) {};
    public default void setConstraintConstants(int pSlot, double kMaxVelocity, double kMaxAcceleration, double kMaxJerk) {};
    public default void stopMotor() {}
}
