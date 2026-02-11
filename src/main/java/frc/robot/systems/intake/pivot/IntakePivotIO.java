// REBELLION 10014

package frc.robot.systems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotInputs {
      public boolean iIsIntakePivotConnected = false;
      public double iIntakePivotVelocityMPS = 0.0;
      public double iIntakePivotAccelerationMPSS = 0.0;
      public double iIntakePivotMotorVolts = 0.0;
      public double iIntakePivotSupplyCurrentAmps = 0.0;
      public double iIntakePivotStatorCurrentAmps = 0.0;
      public double iIntakePivotTempCelsius = 0.0;
    }

    public default void updateInputs(IntakePivotInputs pInputs) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}

}
