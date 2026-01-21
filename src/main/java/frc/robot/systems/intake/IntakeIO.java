// REBELLION 10014

package frc.robot.systems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class ModuleInputs {
      public boolean iIsIntakeConnected = false;
      public double iIntakeStatorCurrentAmps = 0.0;
      public double iIntakeSupplyCurrentAmps = 0.0;
      public double iIntakeTorqueCurrentAmps = 0.0;
      public double iIntakeTemperatureCelsius = 0.0;
      public double iIntakeAppliedVolts = 0.0;
      public double iIntakeMotorVolts = 0.0;
      public double iIntakeRPM = 0.0;
    }

    public default void updateInputs(ModuleInputs inputs) {}

    public default void setIntakeRPM(double pRPM) {}

    public default void setIntakeVolts(double pVolts) {}
}
