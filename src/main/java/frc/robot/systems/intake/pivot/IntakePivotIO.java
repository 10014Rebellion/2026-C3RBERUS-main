// REBELLION 10014

package frc.robot.systems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotInputs {
      public boolean iIsIntakePivotConnected = false;
      public Rotation2d iIntakePivotRotation = Rotation2d.kZero;
      public double iIntakePivotVelocityRPS = 0.0;
      public double iIntakePivotAccelerationRPSS = 0.0;
      public double iIntakePivotMotorVolts = 0.0;
      public double iIntakePivotSupplyCurrentAmps = 0.0;
      public double iIntakePivotStatorCurrentAmps = 0.0;
      public double iIntakePivotTempCelsius = 0.0;
    }

    public default void updateInputs(IntakePivotInputs pInputs) {}

    public default void setMotorVolts(double pVolts) {}

    public default void setMotorRot(Rotation2d pRot, double feedforward) {}

    public default void setPDConstants(double pKP, double pKD) {}

    public default void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {}

    public default void enforceSoftLimits() {}

    public default void stopMotor() {}

}
