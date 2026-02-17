// REBELLION 10014

package frc.robot.systems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
    @AutoLog
    public static class ConveyorInputs {
      public boolean iIsConveyorConnected = false;
      public double iConveyorVelocityRPS = 0.0;
      public double iConveyorAccelerationMPSS = 0.0;
      public double iConveyorMotorVolts = 0.0;
      public double iConveyorSupplyCurrentAmps = 0.0;
      public double iConveyorStatorCurrentAmps = 0.0;
      public double iConveyorTempCelsius = 0.0;
    }

    public default void updateInputs(ConveyorInputs pInputs) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}
}
