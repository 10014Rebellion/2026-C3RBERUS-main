package frc.robot.systems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.MotionMagicFOCControllerFF;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RelativeCANCoderHardware;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.lib.hardware.HardwareRecords.SimpleController;
public class ShooterConstants {

    public static class FuelPumpConstants {

        public static final double kToleranceRPS = 3.0;
        public static final Rotation2d kRPSForShooting = Rotation2d.fromRotations(85);

        public enum FuelPumpStates {
            DISCONNECTED(null),
            INDEXING(() -> Rotation2d.fromRotations(0)), // TODO: TUNE ME!
            UNJAMMING(() -> Rotation2d.fromRotations(0)), // TODO: TUNE ME!
            STOPPED(() -> Rotation2d.fromRotations(0));

            Supplier<Rotation2d> mDesiredRPS;
            FuelPumpStates(Supplier<Rotation2d> pDesiredRPS) {
                this.mDesiredRPS = pDesiredRPS;
            }

            public Supplier<Rotation2d> getRPS() {
                return mDesiredRPS == null ? () -> Rotation2d.kZero : mDesiredRPS;
            }

            public Rotation2d getRotRPS() {
                return mDesiredRPS == null ? Rotation2d.kZero : mDesiredRPS.get();
            }

            public double getValueRPS() {
                return mDesiredRPS == null ? 0 : mDesiredRPS.get().getRotations();
            }
        }

        public static final BasicMotorHardware kFuelPumpLeaderConfig = new BasicMotorHardware(
            53,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(40, 50)
        );

        public static final FollowerMotorHardware kFuelPumpFollowerConfig = new FollowerMotorHardware(
            54,
            kFuelPumpLeaderConfig,
            MotorAlignmentValue.Opposed
        );
    }

    public static class FlywheelConstants {

        public static final double kMaxFlywheelTestedRPS = 112;
        public static final double kToleranceRPS = 3.0;
        public static final double kBangBangTimeout = 0.25;

        public static final RelativeCANCoderHardware kCANCoderConfig = new RelativeCANCoderHardware(
            50,
            1,
            SensorDirectionValue.Clockwise_Positive
        ); 

        public static final BasicMotorHardware kFlywheelLeaderConfig = new BasicMotorHardware(
            51,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(60, 80)
        );

        public static final FollowerMotorHardware kFlywheelFollowerConfig = new FollowerMotorHardware(
            52,
            kFlywheelLeaderConfig,
            MotorAlignmentValue.Opposed
        );

        public static final MotionMagicFOCControllerFF kFlywheelControlConfig = new MotionMagicFOCControllerFF(
            0,
            new PDConstants(10.0, 0), // Tuned for C3RBERUS!
            new SimpleMotorFeedforward(0.37, 0.13, 0), // Tuned for C3RBERUS!
            new MotionMagicConstants(0.0, 1000.0, 0.0)
        );
    }
}
