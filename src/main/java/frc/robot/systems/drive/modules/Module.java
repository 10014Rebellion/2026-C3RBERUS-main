// REBELLION 10014

package frc.robot.systems.drive.modules;

import static frc.robot.systems.drive.DriveConstants.kModuleControllerConfigs;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.telemetry.Telemetry;
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

    private final ModuleIO mIO;
    private final ModuleInputsAutoLogged mInputs = new ModuleInputsAutoLogged();

    private final String kModuleName;

    private Double mVelocitySetpointMPS = null;
    private Double mAmperageFeedforward = null;
    private SimpleMotorFeedforward mDriveFF = DriveConstants.kModuleControllerConfigs.driveFF();

    private Rotation2d mAzimuthSetpointAngle = null;
    private SimpleMotorFeedforward mAzimuthFF = DriveConstants.kModuleControllerConfigs.azimuthFF();

    private SwerveModuleState mCurrentState = new SwerveModuleState();
    private SwerveModulePosition mCurrentPosition = new SwerveModulePosition();

    public Module(String pKey, ModuleIO pIO) {
        this.mIO = pIO;
        kModuleName = "Module/" + pKey;
    }

    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Drive/" + kModuleName, mInputs);

        mCurrentState = new SwerveModuleState(mInputs.iDriveVelocityMPS, mInputs.iAzimuthPosition);
        mCurrentPosition = new SwerveModulePosition(mInputs.iDrivePositionM, mInputs.iAzimuthPosition);

        if (mVelocitySetpointMPS != null) {
            if (mAmperageFeedforward != null) {
                double ffOutput = mDriveFF.calculateWithVelocities(mVelocitySetpointMPS, mAmperageFeedforward);

                Telemetry.log("Drive/" + kModuleName + "/AmperageFeedforward", mAmperageFeedforward);
                Telemetry.log("Drive/" + kModuleName + "/ffOutput", ffOutput);

                mIO.setDriveVelocity(mVelocitySetpointMPS, ffOutput);
            } else {
                mIO.setDriveVelocity(mVelocitySetpointMPS, 0.0);
            }
        
            if (mAzimuthSetpointAngle != null) {
                double ffOutput = mAzimuthFF.calculateWithVelocities(0, 0);
                Telemetry.log("Drive/" + kModuleName + "/SimpleFeedforward", ffOutput);
                mIO.setAzimuthPosition(mAzimuthSetpointAngle, ffOutput);
            }

            if (DriverStation.isDisabled()) stop();

            LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    mIO.setDrivePID(tDriveP.get(), 0.0, tDriveD.get());
                },
                tDriveP,
                tDriveD
            );

            LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    mDriveFF = new SimpleMotorFeedforward(tDriveS.get(), tDriveV.get(), tDriveA.get());
                },
                tDriveS,
                tDriveV,
                tDriveA
            );

            LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    mIO.setAzimuthPID(tTurnP.get(), 0.0, tTurnD.get());
                },
                tTurnP,
                tTurnD
            );

            LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    mAzimuthFF = new SimpleMotorFeedforward(tTurnS.get(), 0.0, 0.0);
                },
                tTurnS
            );
        }
    }

    /* Sets the desired setpoint of the module with FF. Disables the all FF include velocity FF
     * @param state the desired velocity and rotation of the module
     */
    public SwerveModuleState setDesiredState(SwerveModuleState pState) {
        setDesiredStateWithAmpFF(pState, null);
        return getDesiredState();
    }

    /* Sets the desired setpoint of the module with FF
     * @param state the desired velocity and rotation of the module
     * @param ampFeedforward The amperage added to the PID from FF, also enables the PID
     */
    public SwerveModuleState setDesiredStateWithAmpFF(SwerveModuleState pState, Double pAmpFeedforward) {
        setAmpFeedforward(pAmpFeedforward);
        setDesiredVelocity(pState.speedMetersPerSecond);
        setDesiredRotation(pState.angle);
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
        setDesiredRotation(pAzimuthRotation);
        setDesiredVelocity(null);
        setDriveVoltage(pInputVolts);
    }

    /* Sets the velocity of the module
     * @param velocitySetpoint the velocity setpoint
     */
    public void setDesiredVelocity(Double pVelocitySetpoint) {
        mVelocitySetpointMPS = pVelocitySetpoint;
    }

    /* Sets the amperage Feedforward
     * @pararm Rotation2d angleSetpoint
     */
    public void setAmpFeedforward(Double pAmperage) {
        this.mAmperageFeedforward = pAmperage;
    }

    /* Sets azimuth rotation goal
     * @pararm Rotation2d angleSetpoint
     */
    public void setDesiredRotation(Rotation2d pAngleSetpoint) {
        mAzimuthSetpointAngle = pAngleSetpoint;
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

    /* Stops modules by setting voltage to zero */
    public void stop() {
        setDriveVoltage(0.0);
        setAzimuthVoltage(0.0);
    }

    /* Gets the setpoint state of the module(speed and rotation) */
    public SwerveModuleState getDesiredState() {
        return new SwerveModuleState(mVelocitySetpointMPS, mAzimuthSetpointAngle);
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
}
