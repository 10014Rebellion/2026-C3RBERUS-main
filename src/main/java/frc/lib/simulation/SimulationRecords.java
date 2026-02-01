package frc.lib.simulation;

import edu.wpi.first.math.system.plant.DCMotor;

public class SimulationRecords {

    public static record SimulatedElevator(
        DCMotor kMotor,
        double kCarriageMassKg,
        double kDrumRadiusMeters,
        double kMinHeightMeters,
        double kMaxHeightMeters,
        boolean kSimulateGravity,
        double kStartingHeightMeters,
        double kMeasurementStdDevs
    ) {}
}
