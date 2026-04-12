package frc.robot.systems.efi.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.MeasurementHealthValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.hardware.HardwareRecords.CANRangeConfiguration;
import frc.robot.RobotConstants;

public class SensorIOCANRange implements SensorIO{

    private final CANrange kCANRange;

    private final StatusSignal<Distance> distanceFromFuel;
    private final StatusSignal<Boolean> isDetected;
    private final StatusSignal<Distance> distanceFromFuelStdev;
    private final StatusSignal<Double> ambience;
    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<MeasurementHealthValue> measurementHealth;
    private final StatusSignal<Time> measurementTime;

    private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

    public SensorIOCANRange(CANRangeConfiguration configuration){
        kCANRange = new CANrange(configuration.canRangeID(), RobotConstants.kSubsystemsCANBus);

        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 3000;
        canRangeConfig.ProximityParams.ProximityHysteresis = configuration.positionTolerance();
        canRangeConfig.ProximityParams.ProximityThreshold = configuration.fuelDetectionCutoff();

        kCANRange.getConfigurator().apply(canRangeConfig);

        // Intialize Status Signals
        distanceFromFuel = kCANRange.getDistance();
        isDetected = kCANRange.getIsDetected();
        distanceFromFuelStdev = kCANRange.getDistanceStdDev();
        ambience = kCANRange.getAmbientSignal();
        signalStrength = kCANRange.getSignalStrength();
        measurementHealth = kCANRange.getMeasurementHealth();
        measurementTime = kCANRange.getMeasurementTime();
    }

    @SuppressWarnings("unlikely-arg-type")
    @Override
    public void updateInputs(SensorInputs inputs){
        inputs.distanceFromSensor = distanceFromFuel.getValueAsDouble();
        inputs.distanceFromSensorStdDev = distanceFromFuelStdev.getValueAsDouble();
        inputs.hasObject = isDetected.getValue();
        inputs.ambience = ambience.getValueAsDouble();
        inputs.signalStrength = signalStrength.getValueAsDouble();
        inputs.isMeasurementHealthGood = measurementHealth.equals(MeasurementHealthValue.Good);
        inputs.measurementTime = measurementTime.getValueAsDouble();
        
        inputs.isSensorConnected = BaseStatusSignal.refreshAll(
            distanceFromFuel,
            distanceFromFuelStdev,
            isDetected,
            ambience,
            signalStrength,
            measurementHealth,
            measurementTime).isOK();
    }

    @Override
    public boolean getCANRangeValue(){
        return isDetected.getValue();
    }
    
}