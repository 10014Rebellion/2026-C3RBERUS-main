package frc.robot.systems.intake.rack;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.intake.IntakeConstants;

public class IntakeRackSS extends SubsystemBase {
    public static enum IntakeRackState {
        STOPPED, // At rest
        TUNING_VOLTAGE,
        TUNING_AMPS,
        STOW,
        SAFESTOW,
        COMPACT_HIGH,
        COMPACT_LOW,
        COMPACT_AMPS,
        INTAKE,
        TUNING_SETPOINT,
        INVALID
    }

    private final IntakeRackIO mIntakeRackIO;
    private final ElevatorFeedforward mIntakeFF;
    private final IntakeRackInputsAutoLogged mIntakeRackInputs = new IntakeRackInputsAutoLogged();

    private final LoggedTunableNumber tIntakeKP = new LoggedTunableNumber("Intake/Control/PID/kP", 
        IntakeConstants.RackConstants.kRackController.pdController().kP());
    private final LoggedTunableNumber tIntakeKD = new LoggedTunableNumber("Intake/Control/PID/kD", 
        IntakeConstants.RackConstants.kRackController.pdController().kD());
    private final LoggedTunableNumber tIntakeKS = new LoggedTunableNumber("Intake/Control/FF/kS", 
        IntakeConstants.RackConstants.kRackController.feedforward().getKs());
    private final LoggedTunableNumber tIntakeKG = new LoggedTunableNumber("Intake/Control/FF/kG", 
        IntakeConstants.RackConstants.kRackController.feedforward().getKg());
    private final LoggedTunableNumber tIntakeKV = new LoggedTunableNumber("Intake/Control/FF/kV", 
        IntakeConstants.RackConstants.kRackController.feedforward().getKv());
    private final LoggedTunableNumber tIntakeKA = new LoggedTunableNumber("Intake/Control/FF/kA", 
        IntakeConstants.RackConstants.kRackController.feedforward().getKa());
    private final LoggedTunableNumber tIntakeCruiseVelDPS = new LoggedTunableNumber("Intake/Control/Profile/CruiseVelDPS", 
        IntakeConstants.RackConstants.kRackController.motionMagicConstants().maxVelocity() * 360.0);
    private final LoggedTunableNumber tIntakeMaxAccelDPSS = new LoggedTunableNumber("Intake/Control/Profile/MaxAccelerationDPSS", 
        IntakeConstants.RackConstants.kRackController.motionMagicConstants().maxAcceleration() * 360.0);
    private final LoggedTunableNumber tIntakeMaxJerkDPSSS = new LoggedTunableNumber("Intake/Control/Profile/MaxJerkDPSSS", 
        IntakeConstants.RackConstants.kRackController.motionMagicConstants().maxJerk() * 360.0);
    private final LoggedTunableNumber tIntakeToleranceDegrees = new LoggedTunableNumber("Intake/Control/ToleranceMeters", 
        IntakeConstants.RackConstants.kRackToleranceMeters);
  
    @AutoLogOutput(key = "IntakeRack/States/CurrentState")
    private IntakeRackState mCurrentIntakeState = IntakeRackState.STOPPED;

    @AutoLogOutput(key = "IntakeRack/RotationGoal/CurrentGoal")
    private double mGoalMeters = 0.0;
    private int mDesiredDirection = 0;

    @AutoLogOutput(key = "Intake/LimitsEnforced")
    private boolean mLimitEnforced = false;
  
    public IntakeRackSS(IntakeRackIO pIntakeRackIO) {
        this.mIntakeRackIO = pIntakeRackIO;
        this.mIntakeFF = IntakeConstants.RackConstants.kRackController.feedforward();
    }

    @Override
    public void periodic() {
        mIntakeRackIO.updateInputs(mIntakeRackInputs);

        refreshTuneables();
        executeState();

        Logger.processInputs("Intake", mIntakeRackInputs);
    }

    /*
     * Performs variable updates or parameter intializations when a state is set, SHOULD NOT CHANGE THE STATE THROUGH HERE.
     */
    @SuppressWarnings("incomplete-switch")
    private void initializeState(IntakeRackState pStateToInit) {
        mCurrentIntakeState = pStateToInit;
        switch (mCurrentIntakeState) {
            case STOPPED -> {
            } case TUNING_VOLTAGE -> {
            } case COMPACT_AMPS -> {
                mIntakeRackIO.setMotorAmps(IntakeConstants.RackConstants.tConstantCompactAmps.get());
            } case STOW, SAFESTOW, COMPACT_HIGH, COMPACT_LOW, INTAKE, TUNING_SETPOINT -> {
                mIntakeRackIO.resetPPID();
            } case INVALID -> {}
            default -> {
                Telemetry.reportIssue(null);
            }
        }
    }

    /*
     * Runs actions periodically to execute state, SHOULD NOT CHANGE THE STATE THROUGH HERE.
     */
    private void executeState() {
        switch (mCurrentIntakeState) {
            case STOPPED -> {
                mIntakeRackIO.stopMotor();
            } case TUNING_VOLTAGE -> {
                setIntakeVoltage(IntakeConstants.RackConstants.tRackTuningVoltage.get());
            } case TUNING_AMPS -> {
                setIntakeAmps(IntakeConstants.RackConstants.tRackTuningAmp.get());
            } case COMPACT_AMPS -> {
                setIntakeAmps(IntakeConstants.RackConstants.tConstantCompactAmps.get());
            } case STOW, SAFESTOW, COMPACT_HIGH, COMPACT_LOW, INTAKE, TUNING_SETPOINT -> {
                setIntakePosition(IntakeConstants.RackConstants.kStateToSetpointMapIntake.get(mCurrentIntakeState).get());
            } case INVALID -> {}
            default -> {
                Telemetry.reportIssue(null);
            }
        }
    }
    /*
     * Performs variable updates or parameter resets when a state ends, SHOULD NOT CHANGE THE STATE THROUGH HERE.
     */
    @SuppressWarnings("incomplete-switch")
    private void endState(IntakeRackState pStateToEnd) {
        switch (pStateToEnd) {}
    }

    public Command setStateCmd(IntakeRackState pNewState) {
        return setStateCmd(pNewState, true);
    }

    public Command setStateCmd(IntakeRackState pNewState, boolean holdRequirementContinuously) {
        return new FunctionalCommand(
            () -> setState(pNewState), 
            () -> {}, (interrupted) ->  {}, 
            () -> !holdRequirementContinuously, 
            this
        );
    }

    /* SETTERS */
    private void setState(IntakeRackState pNewState) {
        endState(mCurrentIntakeState);
        mCurrentIntakeState = pNewState;
        initializeState(pNewState);
    }

    public void setIntakePosition(double pPositionM) {
        Telemetry.log("IntakeRack/Setpoint/Non-limited", pPositionM);

        pPositionM = clampPositionToSoftLimits(pPositionM);

        Telemetry.log("IntakeRack/Setpoint/Limited", pPositionM);

        mGoalMeters = pPositionM;

        double ffOutput = mIntakeFF.calculate(
            mIntakeRackInputs.iIntakeRackPositionM, 
            mIntakeRackInputs.iIntakeClosedLoopReferenceSlope
        );

        Telemetry.log("Intake/Rack/ffOutput", ffOutput);

        mIntakeRackIO.setMotorPosition(pPositionM, ffOutput);

        mDesiredDirection = toDirection(getErrorPosition().getDegrees());
        enforceSoftLimits();
    }

    public void setIntakeVoltage(double pVolts) {
        mDesiredDirection = toDirection(pVolts);
        mIntakeRackIO.setMotorVolts(pVolts);
        enforceSoftLimits();
    }

    public void setIntakeAmps(double pAmps) {
        mDesiredDirection = toDirection(pAmps);
        mIntakeRackIO.setMotorAmps(pAmps);
        enforceSoftLimits();
    }

    private void setFF(double kS, double kG, double kV, double kA) {
        mIntakeFF.setKs(kS);
        mIntakeFF.setKg(kG);
        mIntakeFF.setKv(kV);
        mIntakeFF.setKa(kA);
    }

    /* GETTERS */
    public IntakeRackState getIntakeState() {
        return mCurrentIntakeState;
    }
  
    @AutoLogOutput(key = "Intake/Feedback/ErrorRotation")
    public Rotation2d getErrorPosition() {
        return Rotation2d.fromRotations(getCurrentGoal() - mIntakeRackInputs.iIntakeRackPositionM);
    }

    @AutoLogOutput(key = "Intake/Feedback/CurrentGoal")
    public double getCurrentGoal() {
        return mGoalMeters;    
    }

    @AutoLogOutput(key = "Intake/Feedback/AtGoal")
    public boolean atGoal() {
        return Math.abs(getErrorPosition().getDegrees()) < tIntakeToleranceDegrees.get();
    }

    /* LOGICCC */
    private double clampPositionToSoftLimits(double pPositionToClampM) {
        return    
        MathUtil.clamp(
                pPositionToClampM,
                IntakeConstants.RackConstants.kRackLimitsMeters.backwardLimitM(),
                IntakeConstants.RackConstants.kRackLimitsMeters.forwardLimitM()
            );
    }

    private void refreshTuneables() {
        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> mIntakeRackIO.setPDConstants(tIntakeKP.get(), tIntakeKD.get()), 
            tIntakeKP, tIntakeKD
        );

        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> setFF(tIntakeKS.get(), tIntakeKG.get(), tIntakeKV.get(), tIntakeKA.get()), 
            tIntakeKS, tIntakeKG, tIntakeKV, tIntakeKA
        );
  
        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> mIntakeRackIO.setMotionMagicConstants(
                tIntakeCruiseVelDPS.get() / 360.0, 
                tIntakeMaxAccelDPSS.get() / 360.0, 
                tIntakeMaxJerkDPSSS.get() / 360.0), 
            tIntakeCruiseVelDPS, tIntakeMaxAccelDPSS, tIntakeMaxJerkDPSSS
        );
    }

    public void enforceSoftLimits() {
        if(
        (mIntakeRackInputs.iIntakeRackPositionM > IntakeConstants.RackConstants.kRackLimitsMeters.forwardLimitM()
            && mDesiredDirection == 1 ) 
            || 
        (mIntakeRackInputs.iIntakeRackPositionM < IntakeConstants.RackConstants.kRackLimitsMeters.backwardLimitM() 
            && mDesiredDirection == -1)) {
                mLimitEnforced = true;
                mIntakeRackIO.stopMotor();
        } else {
            mLimitEnforced = false;
        }
    }

    public int toDirection(double val) {
        if(val > 0) return 1;
        if(val < 0) return -1;
        else return 0;
    }
}