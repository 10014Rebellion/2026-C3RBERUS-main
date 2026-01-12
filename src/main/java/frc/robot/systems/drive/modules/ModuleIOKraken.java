// REBELLION 10014

package frc.robot.systems.drive.modules;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.systems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.systems.drive.DriveConstants.ModuleHardwareConfig;

public class ModuleIOKraken implements ModuleIO {
    private final TalonFX mDriveMotor;
    private final VelocityTorqueCurrentFOC mDriveControl = new VelocityTorqueCurrentFOC(0.0);
    private final VoltageOut mDriveVoltageControl = new VoltageOut(0.0);
    private final double mDriveAppliedVolts = 0.0;

    private final StatusSignal<Angle> mDrivePositionM;
    private final StatusSignal<AngularVelocity> mDriveVelocityMPS;
    private final StatusSignal<Voltage> mDriveVoltage;
    private final StatusSignal<Current> mDriveSupplyCurrent;
    private final StatusSignal<Current> mDriveStatorCurrent;
    private final StatusSignal<Current> mDriveTorqueCurrent;
    private final StatusSignal<Temperature> mDriveTempCelsius;
    private final StatusSignal<AngularAcceleration> mDriveAccelerationMPSS;

    private final TalonFX mAzimuthMotor;
    private final PositionDutyCycle mAzimuthPositionControl = new PositionDutyCycle(0.0);
    private final VoltageOut mAzimuthVoltageControl = new VoltageOut(0.0);
    private final double mAzimuthAppliedVolts = 0.0;

    private final StatusSignal<Angle> mAzimuthPosition;
    private final StatusSignal<AngularVelocity> mAzimuthVelocity;
    private final StatusSignal<Voltage> mAzimuthVoltage;
    private final StatusSignal<Current> mAzimuthStatorCurrent;
    private final StatusSignal<Current> mAzimuthSupplyCurrent;
    private final StatusSignal<Temperature> mAzimuthTemp;

    private final CANcoder mAbsoluteEncoder;
    private final StatusSignal<Angle> mAbsolutePositionSignal;

    public ModuleIOKraken(ModuleHardwareConfig pConfig) {
        mDriveMotor = new TalonFX(pConfig.driveID(), kCANBus);
        var driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = kDriveStatorAmpLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveSupplyAmpLimit;

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

        driveConfig.Voltage.PeakForwardVoltage = kPeakVoltage;
        driveConfig.Voltage.PeakReverseVoltage = -kPeakVoltage;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = kDriveMotorGearing / kWheelCircumferenceMeters;

        driveConfig.Slot0.kP = kModuleControllerConfigs.driveController().getP();
        driveConfig.Slot0.kI = kModuleControllerConfigs.driveController().getI();
        driveConfig.Slot0.kD = kModuleControllerConfigs.driveController().getD();
        mDrivePositionM = mDriveMotor.getPosition();
        mDriveVelocityMPS = mDriveMotor.getVelocity();
        mDriveVoltage = mDriveMotor.getMotorVoltage();
        mDriveSupplyCurrent = mDriveMotor.getSupplyCurrent();
        mDriveStatorCurrent = mDriveMotor.getStatorCurrent();
        mDriveTorqueCurrent = mDriveMotor.getTorqueCurrent();
        mDriveTempCelsius = mDriveMotor.getDeviceTemp();
        mDriveAccelerationMPSS = mDriveMotor.getAcceleration();

        mDriveMotor.getConfigurator().apply(driveConfig);
        
        mAbsoluteEncoder = new CANcoder(pConfig.encoderID(), kCANBus);
        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(-pConfig.offset()));
        mAbsoluteEncoder.getConfigurator().apply(encoderConfig);
        
        mAbsolutePositionSignal = mAbsoluteEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAbsolutePositionSignal);
        mAbsoluteEncoder.optimizeBusUtilization();

        /* AZIMUTH INSTANTIATION AND CONFIGURATION */
        mAzimuthMotor = new TalonFX(pConfig.azimuthID(), kCANBus);
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit = kAzimuthStatorAmpLimit;
        turnConfig.Voltage.PeakForwardVoltage = kPeakVoltage;
        turnConfig.Voltage.PeakReverseVoltage = -kPeakVoltage;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnConfig.MotorOutput.Inverted = kTurnMotorInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        turnConfig.Feedback.FeedbackRemoteSensorID = mAbsoluteEncoder.getDeviceID();
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.SensorToMechanismRatio = kCANCoderToMechanismRatio;
        turnConfig.Feedback.RotorToSensorRatio = kAzimuthMotorGearing;
        turnConfig.Slot0.kP = kModuleControllerConfigs.azimuthController().getP();
        turnConfig.Slot0.kD = kModuleControllerConfigs.azimuthController().getD();
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = kAzimuthFOCAmpLimit;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kAzimuthFOCAmpLimit;

        mAzimuthMotor.getConfigurator().apply(turnConfig);

        mAzimuthPosition = mAzimuthMotor.getPosition();
        mAzimuthVelocity = mAzimuthMotor.getVelocity();
        mAzimuthVoltage = mAzimuthMotor.getMotorVoltage();
        mAzimuthStatorCurrent = mAzimuthMotor.getStatorCurrent();
        mAzimuthSupplyCurrent = mAzimuthMotor.getSupplyCurrent();
        mAzimuthTemp = mAzimuthMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(ModuleInputs pInputs) {
        pInputs.iIsDriveConnected = BaseStatusSignal.refreshAll(
            mDriveVelocityMPS,
            mDrivePositionM,
            mDriveVoltage,
            mDriveSupplyCurrent,
            mDriveStatorCurrent,
            mDriveTorqueCurrent,
            mDriveTempCelsius
        ).isOK();
        pInputs.iDrivePositionM = (mDrivePositionM.getValueAsDouble());
        pInputs.iDriveVelocityMPS = (mDriveVelocityMPS.getValueAsDouble());
        pInputs.iDriveAppliedVolts = mDriveAppliedVolts;
        pInputs.iDriveMotorVolts = mDriveVoltage.getValueAsDouble();
        pInputs.iDriveSupplyCurrentAmps = mDriveSupplyCurrent.getValueAsDouble();
        pInputs.iDriveStatorCurrentAmps = mDriveStatorCurrent.getValueAsDouble();
        pInputs.iDriveTorqueCurrentAmps = mDriveTorqueCurrent.getValueAsDouble();
        pInputs.iDriveTemperatureCelsius = mDriveTempCelsius.getValueAsDouble();
        pInputs.iDriveAccelerationMPSS = mDriveAccelerationMPSS.getValueAsDouble();

        pInputs.iIsAzimuthConnected = BaseStatusSignal.refreshAll(
                        mAzimuthVelocity,
                        mAzimuthVoltage,
                        mAzimuthStatorCurrent,
                        mAzimuthSupplyCurrent,
                        mAzimuthTemp,
                        mAzimuthPosition)
                .isOK();
        pInputs.iAzimuthPosition = Rotation2d.fromRotations(mAzimuthPosition.getValueAsDouble());
        pInputs.iAzimuthVelocity = Rotation2d.fromRotations(mAzimuthVelocity.getValueAsDouble());
        pInputs.iAzimuthAppliedVolts = mAzimuthAppliedVolts;
        pInputs.iAzimuthMotorVolts = mAzimuthVoltage.getValueAsDouble();
        pInputs.iAzimuthStatorCurrentAmps = mAzimuthStatorCurrent.getValueAsDouble();
        pInputs.iAzimuthSupplyCurrentAmps = mAzimuthSupplyCurrent.getValueAsDouble();
        pInputs.iAzimuthTemperatureCelsius = mAzimuthTemp.getValueAsDouble();

        pInputs.iIsCancoderConnected = BaseStatusSignal.refreshAll(mAbsolutePositionSignal).isOK();
        pInputs.iAzimuthAbsolutePosition = Rotation2d.fromRotations(mAbsolutePositionSignal.getValueAsDouble());
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double pVolts) {
        mDriveMotor.setControl(mDriveVoltageControl.withOutput(pVolts));
    }

    @Override
    public void setDriveAmperage(double pAmps) {
        mDriveMotor.setControl(new TorqueCurrentFOC(pAmps));
    }

    @Override
    public void setDriveVelocity(double pVelocityMPS, double pFeedforward) {
        mDriveMotor.setControl(mDriveControl.withVelocity(pVelocityMPS).withFeedForward(pFeedforward));
    }

    @Override
    public void setDrivePID(double pKP, double pKI, double pKD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kI = pKI;
        slotConfig.kD = pKD;
        mDriveMotor.getConfigurator().apply(slotConfig);
    }

    /////////// CANCODER METHODS \\\\\\\\\\\
    @Override
    public void resetAzimuthEncoder() {
        return;
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double pVolts) {
        /* Sets azimuth voltage inbetween kPeakVoltage and -kPeakVoltage */
        mAzimuthMotor.setControl(mAzimuthVoltageControl.withOutput(pVolts));
    }

    @Override
    public void setAzimuthPosition(Rotation2d pRotation, double pFeedforward) {
        /* Uses voltage PID with a arbitrary FF on the with Slot 0 gains */
        mAzimuthMotor.setControl(mAzimuthPositionControl
                .withPosition(pRotation.getRotations())
                .withFeedForward(pFeedforward)
                .withSlot(0));
    }

    /* Sets azimuth PID values on slot 0, used for tunable numbers */
    @Override
    public void setAzimuthPID(double pKP, double pKI, double pKD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kI = pKI;
        slotConfig.kD = pKD;
        mAzimuthMotor.getConfigurator().apply(slotConfig);
    }
}
