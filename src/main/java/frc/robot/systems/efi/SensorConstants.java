package frc.robot.systems.efi;

import frc.lib.hardware.HardwareRecords.CANRangeConfiguration;

public class SensorConstants {
    public static final int leftCANRangeID = 0;
    public static final int midCANRangeID = 0;
    public static final int rightCANRangeID = 0;
    public static final double kPositionTolerance = 0;
    public static final double kFuelCutoff = 0;

    public static CANRangeConfiguration leftCANRangeConfiguration = new CANRangeConfiguration(leftCANRangeID, kFuelCutoff, kPositionTolerance);
    public static CANRangeConfiguration midCANRangeConfiguration = new CANRangeConfiguration(midCANRangeID, kFuelCutoff, kPositionTolerance);
    public static CANRangeConfiguration rightCANRangeConfiguration = new CANRangeConfiguration(rightCANRangeID, kFuelCutoff, kPositionTolerance);
}