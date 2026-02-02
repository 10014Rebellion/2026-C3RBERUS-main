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
    public default void setMotorPosition(double pPositionM, double pFeedforward) {}
    public default void setPIDConstants(double kP, double kI, double kD) {};
    public default void setSlot(int pSlot) {};
    public default void setConstraintConstants(double kMaxVelocity, double kMaxAcceleration, double kMaxJerk) {};
    public default void stopMotor() {}
}
