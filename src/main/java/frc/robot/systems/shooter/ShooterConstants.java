package frc.robot.systems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.SimpleController;

public class ShooterConstants {
    public static class IndexerConstants {
        public static final BasicMotorHardware kIndexerLeaderConfig = new BasicMotorHardware(
            54,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(40, 50)
        );

        public static final FollowerMotorHardware kIndexerFollowerConfig = new FollowerMotorHardware(
            55,
            kIndexerLeaderConfig,
            MotorAlignmentValue.Opposed
        );
    }

    public static class Hood {

    }

    public static class FlywheelConstants {
        public static final BasicMotorHardware kFlywheelLeaderConfig = new BasicMotorHardware(
            53,
            Constants.kSubsystemsCANBus,
            1,
            InvertedValue.CounterClockwise_Positive,
            NeutralModeValue.Coast,
            new CurrentLimits(40, 50)
        );

        public static final FollowerMotorHardware kFlywheelFollowerConfig = new FollowerMotorHardware(
            54,
            kFlywheelLeaderConfig,
            MotorAlignmentValue.Opposed
        );

        public static final SimpleController kFlywheelControlConfig = new SimpleController(
            0,
            new PDConstants(0, 0),
            new SimpleMotorFeedforward(0, 0)
        );
    }
}
