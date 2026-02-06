package frc.robot.systems.shooter.indexers;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerInputs {
      public boolean iIsIndexerConnected = false;
      public String iIndexerControlMode = "";
      public double iIndexerVelocityRPS = 0.0;
      public double iIndexerAccelerationRPSS = 0.0;
      public double iIndexerMotorVolts = 0.0;
      public double iIndexerSupplyCurrentAmps = 0.0;
      public double iIndexerStatorCurrentAmps = 0.0;
      public double iIndexerTempCelsius = 0.0;
    }

    public default void setPDConstants(double pKP, double pKD) {}

    public default void enforceFollower() {}

    public default void updateInputs(IndexerInputs pInputs) {}

    public default void setMotorVelocity(double pVelocityRPS, double pFeedforward) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}

}
