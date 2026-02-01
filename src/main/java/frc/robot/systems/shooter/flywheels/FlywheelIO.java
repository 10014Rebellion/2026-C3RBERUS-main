// REBELLION 10014

package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelInputs {
      public boolean iIsFlywheelConnected = false;
      public boolean iIsLeader = true;
      public String iFlywheelControlMode = "";
      public double iFlywheelVelocityRPS = 0.0;
      public double iFlywheelAccelerationRPSS = 0.0;
      public double iFlywheelMotorVolts = 0.0;
      public double iFlywheelSupplyCurrentAmps = 0.0;
      public double iFlywheelStatorCurrentAmps = 0.0;
      public double iFlywheelTempCelsius = 0.0;
    }

    public default void updateInputs(FlywheelInputs pInputs) {}

    public default void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}

}
