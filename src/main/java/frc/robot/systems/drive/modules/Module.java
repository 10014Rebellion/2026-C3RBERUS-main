// REBELLION 10014

package frc.robot.systems.drive.modules;

import static frc.robot.systems.drive.DriveConstants.kModuleControllerConfigs;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
    public static final LoggedTunableNumber tDriveP = new LoggedTunableNumber("Module/Drive/kP", kModuleControllerConfigs.driveController().getP());
    public static final LoggedTunableNumber tDriveD = new LoggedTunableNumber("Module/Drive/kD", kModuleControllerConfigs.driveController().getD());
    public static final LoggedTunableNumber tDriveS = new LoggedTunableNumber("Module/Drive/kS", kModuleControllerConfigs.driveFF().getKs());
    public static final LoggedTunableNumber tDriveV = new LoggedTunableNumber("Module/Drive/kV", kModuleControllerConfigs.driveFF().getKv());
    public static final LoggedTunableNumber tDriveA = new LoggedTunableNumber("Module/Drive/kA", kModuleControllerConfigs.driveFF().getKa());

    public static final LoggedTunableNumber tTurnP = new LoggedTunableNumber("Module/AzimuthP", kModuleControllerConfigs.azimuthController().getP());
    public static final LoggedTunableNumber tTurnD = new LoggedTunableNumber("Module/AzimuthD", kModuleControllerConfigs.azimuthController().getD());
    public static final LoggedTunableNumber tTurnS = new LoggedTunableNumber("Module/AzimuthS", kModuleControllerConfigs.azimuthFF().getKs());
    public static final LoggedTunableNumber tTurnV = new LoggedTunableNumber("Module/AzimuthV", kModuleControllerConfigs.azimuthFF().getKv());

    private final ModuleIO mIO;
    private final ModuleInputsAutoLogged mInputs = new ModuleInputsAutoLogged();

    private final String kModuleName;

    private OptionalDouble mVelocitySetpointMPS = OptionalDouble.empty();
    private double mAccelFeedforward = 0.0;
    private SimpleMotorFeedforward mDriveFF = DriveConstants.kModuleControllerConfigs.driveFF();

    private Optional<Rotation2d> mAzimuthSetpointAngle = Optional.empty();
    private Rotation2d mAzimuthSetpointAngularVelocity = Rotation2d.kZero;
    private SimpleMotorFeedforward mAzimuthFF = DriveConstants.kModuleControllerConfigs.azimuthFF();

    private SwerveModuleState mCurrentState = new SwerveModuleState();
    private SwerveModulePosition mCurrentPosition = new SwerveModulePosition();

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(String pKey, ModuleIO pIO) {
        this.mIO = pIO;
        kModuleName = "Module/" + pKey;
    }

    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Drive/" + kModuleName, mInputs);

        mCurrentState = new SwerveModuleState(mInputs.iDriveVelocityMPS, mInputs.iAzimuthPosition);
        mCurrentPosition = new SwerveModulePosition(mInputs.iDrivePositionM, mInputs.iAzimuthPosition);

        /* Works because signals are sampled together */
        int sampleCount = mInputs.odometryTimestamps.length;
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            odometryPositions[i] = new SwerveModulePosition(
                mInputs.odometryDrivePositionsM[i], 
                mInputs.odometryTurnPositions[i]);
        }

        if (mVelocitySetpointMPS.isPresent()) {
            double ffOutput = 
                  mDriveFF.getKs() * Math.signum(mVelocitySetpointMPS.getAsDouble())
                + mDriveFF.getKv() * mVelocitySetpointMPS.getAsDouble()
                + mDriveFF.getKa() * mAccelFeedforward;

            mIO.setDriveVelocity(mVelocitySetpointMPS.getAsDouble(), ffOutput);

            Logger.recordOutput("Drive/" + kModuleName + "/VelocitySetpointMPS", mVelocitySetpointMPS.getAsDouble());
            Logger.recordOutput("Drive/" + kModuleName + (
                (DriveConstants.kUseVoltageFeedforward) ? 
                    "/AmperageFeedforward" : "/VoltageFeedforward"), 
                mAccelFeedforward);
            Logger.recordOutput("Drive/" + kModuleName + "/ffOutput", ffOutput);
        }
        
        if (mAzimuthSetpointAngle.isPresent()) {
            double ffOutput = mAzimuthFF.calculate(mAzimuthSetpointAngularVelocity.getRadians());
            mIO.setAzimuthPosition(mAzimuthSetpointAngle.get(), ffOutput);

            Logger.recordOutput("Drive/" + kModuleName + "/SimpleFeedforwardAzimuth", ffOutput);
            Logger.recordOutput("Drive/" + kModuleName + "/DesiredAzimuthRotationSpeed", mAzimuthSetpointAngularVelocity);
        }
        

        if (DriverStation.isDisabled()) stop();

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                mIO.setDrivePID(
                    tDriveP.get(), 
                    0.0, 
                    tDriveD.get());
            },
            tDriveP,
            tDriveD
        );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                mDriveFF = new SimpleMotorFeedforward(
                    tDriveS.get(), 
                    tDriveV.get(), 
                    tDriveA.get());
            },
            tDriveS,
            tDriveV,
            tDriveA
        );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                mIO.setAzimuthPID(
                    tTurnP.get(), 
                    0.0, 
                    tTurnD.get());
            },
            tTurnP,
            tTurnD
        );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                mAzimuthFF = new SimpleMotorFeedforward(
                    tTurnS.get(), 
                    tTurnV.get(), 
                    0.0);
            },
            tTurnS,
            tTurnV
        );
    }

    /* Sets the desired setpoint of the module with FF. Disables the all FF include velocity FF
     * @param state the desired velocity and rotation of the module
     */
    public SwerveModuleState setDesiredState(SwerveModuleState pState) {
        setDesiredStateWithFF(pState, 0.0);
        return getDesiredState();
    }

    /* Sets the desired setpoint of the module with FF
     * @param state the desired velocity and rotation of the module
     * @param Feedforward The amperage added to the PID from FF, also enables the PID
     */
    public SwerveModuleState setDesiredStateWithFF(SwerveModuleState pState, double pAccelFeedforward) {
        setDesiredStateWithFF(pState, pAccelFeedforward, Rotation2d.kZero);
        return getDesiredState();
    }

    /* Sets the desired setpoint of the module with FF
     * @param state the desired velocity and rotation of the module
     * @param Feedforward The amperage added to the PID from FF, also enables the PID
     */
    public SwerveModuleState setDesiredStateWithFF(SwerveModuleState pState, double pFeedforward, Rotation2d pAzimuthSetpointAngularVelocity) {
        setDesiredVelocity(OptionalDouble.of(pState.speedMetersPerSecond));
        setAccelFeedforward(pFeedforward);
        setDesiredRotation(Optional.of(pState.angle));
        setDesiredAzimuthVelocity(pAzimuthSetpointAngularVelocity);
        return getDesiredState();
    }

    /* Runs characterization of by setting motor drive voltage and rotates the module forward
     * With no closed loop
     * @param inputVolts -12  to 12
     */
    public void runCharacterization(double pInputVolts) {
        runCharacterization(pInputVolts, new Rotation2d());
    }

    /* Runs characterization of by setting motor drive voltage and the rotation the module at a specific rotation
     * With no closed loop
     * @param inputVolts -12  to 12
     * @param azimuthRotation Rotation at which the robot is running characterization voltage at
     */
    public void runCharacterization(double pInputVolts, Rotation2d pAzimuthRotation) {
        setDesiredRotation(Optional.of(pAzimuthRotation));
        setDesiredVelocity(OptionalDouble.empty());
        setDriveVoltage(pInputVolts);
    }

    /* Sets the velocity of the module
     * @param velocitySetpoint the velocity setpoint
     */
    public void setDesiredVelocity(OptionalDouble pVelocitySetpoint) {
        mVelocitySetpointMPS = pVelocitySetpoint;
    }

    /* Sets desired accel for feedforwards
     * @pararm Rotation2d Desired angular velocity for feedforward
     */
    public void setAccelFeedforward(double pAmperage) {
        this.mAccelFeedforward = pAmperage;
    }

    /* Sets azimuth rotation goal
     * @pararm Rotation2d angleSetpoint
     */
    public void setDesiredRotation(Optional<Rotation2d> pAngleSetpoint) {
        mAzimuthSetpointAngle = pAngleSetpoint;
    }

    /* Sets azimuth rotation goal
     * @pararm Rotation2d Desired angular velocity for feedforward
     */
    public void setDesiredAzimuthVelocity(Rotation2d pAzimuthSetpointAngularVelocity) {
        mAzimuthSetpointAngularVelocity = pAzimuthSetpointAngularVelocity;
    }

    /* Sets drive motor's voltage
     * @param driveVolts: -kPeakVoltage to PeakVoltage volts
     */
    public void setDriveVoltage(double pDriveVolts) {
        mIO.setDriveVolts(pDriveVolts);
    }

    /* Sets drive motor's voltage
     * @param driveVolts: -kDriveFOCAmpLimit to kDriveFOCAmpLimit volts
     */
    public void setDriveAmperage(double pAmps) {
        mIO.setDriveAmperage(pAmps);
    }

    /* Sets drive motor's voltage
     * @param azimuthVolts: -kPeakVoltage to PeakVoltage volts
     */
    public void setAzimuthVoltage(double pAzimuthVolts) {
        mIO.setAzimuthVolts(pAzimuthVolts);
    }

    /* Sets drive motor's voltage
     * @param azimuthVolts: -kAzimuthFOCAmpLimit to kAzimuthFOCAmpLimit amps
     */
    public void setAzimuthAmps(double pAzimuthAmps) {
        mIO.setAzimuthAmps(pAzimuthAmps);
    }

    /* Stops modules by setting voltage to zero */
    public void stop() {
        setDriveVoltage(0.0);
        setAzimuthVoltage(0.0);
    }

    /* Gets the setpoint state of the module(speed and rotation) */
    public SwerveModuleState getDesiredState() {
        return (mVelocitySetpointMPS.isEmpty() || mAzimuthSetpointAngle.isEmpty()) 
            ? new SwerveModuleState()
            : new SwerveModuleState(
                mVelocitySetpointMPS.getAsDouble(), 
                mAzimuthSetpointAngle.get());
    }

    /* Gets the physical state of the module(speed and rotation) */
    public SwerveModuleState getCurrentState() {
        return mCurrentState;
    }

    /* Gets the physical position of the module(position and rotation) */
    public SwerveModulePosition getCurrentPosition() {
        return mCurrentPosition;
    }

    /* All logged hardware data in the module */
    public ModuleInputsAutoLogged getInputs() {
        return mInputs;
    }

    /* Resets azimuth encoder from CANCoder */
    public void resetAzimuthEncoder() {
        mIO.resetAzimuthEncoder();
    }

    public double[] getOdometryTimeStamps() {
        return mInputs.odometryTimestamps;
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public String getModuleName() {
        return kModuleName;
    }
}
