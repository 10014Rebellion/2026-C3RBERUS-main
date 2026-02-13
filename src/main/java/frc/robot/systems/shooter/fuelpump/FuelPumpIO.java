package frc.robot.systems.shooter.fuelpump;

import org.littletonrobotics.junction.AutoLog;

public interface FuelPumpIO {
    @AutoLog
    public static class FuelPumpInputs {
      public boolean iIsFuelPumpConnected = false;
      public String iFuelPumpControlMode = "";
      public double iFuelPumpVelocityRPS = 0.0;
      public double iFuelPumpAccelerationRPSS = 0.0;
      public double iFuelPumpMotorVolts = 0.0;
      public double iFuelPumpSupplyCurrentAmps = 0.0;
      public double iFuelPumpStatorCurrentAmps = 0.0;
      public double iFuelPumpTempCelsius = 0.0;
      public double iFuelPumpVelocityGoal = 0.0;
    }

    public default void setPDConstants(int pSlot, double pKP, double pKD) {}

    public default void enforceFollower() {}

    public default void updateInputs(FuelPumpInputs pInputs) {}

    public default void setMotorVelocity(double pVelocityRPS, double pFeedforward) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}

}
