package frc.robot.systems.intake.pivot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.intake.IntakeConstants;

public class IntakePivotSS extends SubsystemBase {
    public static enum IntakePivotStates {
        STOPPED, // At rest
        TUNING_VOLTAGE,
        TUNING_AMPS,
        STOW,
        COMPACT,
        INTAKE,
        TUNING_SETPOINT
    }

    private final IntakePivotIO mIntakePivotIO;
    private final ArmFeedforward mIntakeFF;
    private final IntakePivotInputsAutoLogged mIntakeInputs = new IntakePivotInputsAutoLogged();

    private final LoggedTunableNumber tIntakeKP = new LoggedTunableNumber("Shooter/Intake/Control/PID/kP", 
        IntakeConstants.PivotConstants.kPivotController.pdController().kP());
    private final LoggedTunableNumber tIntakeKD = new LoggedTunableNumber("Shooter/Intake/Control/PID/kD", 
        IntakeConstants.PivotConstants.kPivotController.pdController().kD());
    private final LoggedTunableNumber tIntakeKS = new LoggedTunableNumber("Shooter/Intake/Control/FF/kS", 
        IntakeConstants.PivotConstants.kPivotController.feedforward().getKs());
    private final LoggedTunableNumber tIntakeKG = new LoggedTunableNumber("Shooter/Intake/Control/FF/kG", 
        IntakeConstants.PivotConstants.kPivotController.feedforward().getKg());
    private final LoggedTunableNumber tIntakeKV = new LoggedTunableNumber("Shooter/Intake/Control/FF/kV", 
        IntakeConstants.PivotConstants.kPivotController.feedforward().getKv());
    private final LoggedTunableNumber tIntakeKA = new LoggedTunableNumber("Shooter/Intake/Control/FF/kA", 
        IntakeConstants.PivotConstants.kPivotController.feedforward().getKa());
    private final LoggedTunableNumber tIntakeCruiseVel = new LoggedTunableNumber("Shooter/Intake/Control/Profile/CruiseVel", 
        IntakeConstants.PivotConstants.kPivotController.motionMagicConstants().maxVelocity());
    private final LoggedTunableNumber tIntakeMaxAccel = new LoggedTunableNumber("Shooter/Intake/Control/Profile/MaxAcceleration", 
        IntakeConstants.PivotConstants.kPivotController.motionMagicConstants().maxAcceleration());
    private final LoggedTunableNumber tIntakeMaxJerk = new LoggedTunableNumber("Shooter/Intake/Control/Profile/MaxJerk", 
        IntakeConstants.PivotConstants.kPivotController.motionMagicConstants().maxJerk());
    private final LoggedTunableNumber tIntakeTolerance = new LoggedTunableNumber("Shooter/Intake/Control/ToleranceDegrees", 
        IntakeConstants.PivotConstants.kPivotMotorToleranceRotations.getDegrees());
  
    @AutoLogOutput(key = "Shooter/Intake/States/CurrentState")
    private IntakePivotStates mCurrentIntakeState = IntakePivotStates.STOPPED;

    @AutoLogOutput(key = "Shooter/Intake/RotationGoal/CurrentGoal")
    private Rotation2d mGoalAngle = Rotation2d.kZero;
    private int mDesiredDirection = 0;

    @AutoLogOutput(key = "Shooter/Intake/LimitsEnforced")
    private boolean mLimitEnforced = false;
  
    public IntakePivotSS(IntakePivotIO pIntakePivotIO) {
        this.mIntakePivotIO = pIntakePivotIO;
        this.mIntakeFF = IntakeConstants.PivotConstants.kPivotController.feedforward();
    }

    @Override
    public void periodic() {
        mIntakePivotIO.updateInputs(mIntakeInputs);

        refreshTuneables();
        executeState();

        Logger.processInputs("Intake", mIntakeInputs);
    }

    /*
     * Performs variable updates or parameter intializations when a state is set, SHOULD NOT CHANGE THE STATE THROUGH HERE.
     */
    @SuppressWarnings("incomplete-switch")
    private void initializeState(IntakePivotStates pStateToInit) {
        mCurrentIntakeState = pStateToInit;
    }

    /*
     * Runs actions periodically to execute state, SHOULD NOT CHANGE THE STATE THROUGH HERE.
     */
    private void executeState() {
        switch (mCurrentIntakeState) {
            case STOPPED -> {
                mIntakePivotIO.stopMotor();
            } case TUNING_VOLTAGE -> {
                setIntakeVoltage(IntakeConstants.PivotConstants.tPivotTuningVoltage.get());
            } case TUNING_AMPS -> {
                setIntakeAmps(IntakeConstants.PivotConstants.tPivotTuningAmp.get());
            } case STOW, COMPACT, INTAKE, TUNING_SETPOINT -> {
                setIntakePosition(IntakeConstants.PivotConstants.kStateToSetpointMapHood.get(mCurrentIntakeState).get());
            } default -> {
                Telemetry.reportIssue(null);
            }
        }
    }

    /*
     * Performs variable updates or parameter resets when a state ends, SHOULD NOT CHANGE THE STATE THROUGH HERE.
     */
    @SuppressWarnings("incomplete-switch")
    private void endState(IntakePivotStates pStateToEnd) {
        switch (pStateToEnd) {}
    }

    public Command setStateCmd(IntakePivotStates pNewState) {
        return setStateCmd(pNewState, true);
    }

    public Command setStateCmd(IntakePivotStates pNewState, boolean holdRequirementContinuously) {
        return new FunctionalCommand(
            () -> setState(pNewState), 
            () -> {}, (interrupted) ->  {}, 
            () -> !holdRequirementContinuously, 
            this
        );
    }

    private void setState(IntakePivotStates pNewState) {
        endState(mCurrentIntakeState);
        mCurrentIntakeState = pNewState;
        initializeState(pNewState);
    }

    public void setIntakePosition(Rotation2d pRot) {
        Telemetry.log("IntakePivot/Setpoint/Non-limited", pRot);

        pRot = clampRotToSoftLimits(pRot);

        Telemetry.log("IntakePivot/Setpoint/Limited", pRot);

        mGoalAngle = pRot;

        double ffOutput = mIntakeFF.calculate(
            mIntakeInputs.iEncoderPosition.getRadians(), 
            mIntakeInputs.iIntakeClosedLoopReferenceSlope.getRadians()
        );

        Telemetry.log("Intake/Pivot/ffOutput", ffOutput);

        mIntakePivotIO.setMotorPosition(pRot, ffOutput);

        mDesiredDirection = toDirection(getErrorPositionRotations());
        enforceSoftLimits();
    }

    public void setIntakeVoltage(double pVolts) {
        mDesiredDirection = toDirection(pVolts);
        mIntakePivotIO.setMotorVolts(pVolts);
        enforceSoftLimits();
    }

    public void setIntakeAmps(double pAmps) {
        mDesiredDirection = toDirection(pAmps);
        mIntakePivotIO.setMotorAmps(pAmps);
        enforceSoftLimits();
    }

    public void enforceSoftLimits() {
        if(
        (mIntakeInputs.iEncoderPosition.getRotations() > IntakeConstants.PivotConstants.kPivotLimits.forwardLimit().getRotations()
            && mDesiredDirection == 1 ) 
            || 
        (mIntakeInputs.iEncoderPosition.getRotations() < IntakeConstants.PivotConstants.kPivotLimits.backwardLimit().getRotations() 
            && mDesiredDirection == -1)) {
                mLimitEnforced = true;
                mIntakePivotIO.stopMotor();
        } else {
            mLimitEnforced = false;
        }
    }

    public int toDirection(double val) {
        if(val > 0) return 1;
        if(val < 0) return -1;
        else return 0;
    }

    private void setFF(double kS, double kG, double kV, double kA) {
        mIntakeFF.setKs(kS);
        mIntakeFF.setKg(kG);
        mIntakeFF.setKv(kV);
        mIntakeFF.setKa(kA);
    }

    public IntakePivotStates getIntakeState() {
        return mCurrentIntakeState;
    }
  
    @AutoLogOutput(key = "Shooter/Intake/Feedback/ErrorRotation")
    public double getErrorPositionRotations() {
        return getCurrentGoal().getRotations() - mIntakeInputs.iIntakePivotRotation.getRotations();
    }

    @AutoLogOutput(key = "Shooter/Intake/Feedback/CurrentGoal")
    public Rotation2d getCurrentGoal() {
        return mGoalAngle;    
    }

    @AutoLogOutput(key = "Shooter/Intake/Feedback/AtGoal")
    public boolean atGoal() {
        return Math.abs(getErrorPositionRotations()) < tIntakeTolerance.get();
    }

    private Rotation2d clampRotToSoftLimits(Rotation2d pRotToClamp) {
        return Rotation2d.fromRotations(
            MathUtil.clamp(
                pRotToClamp.getRotations(),
                IntakeConstants.PivotConstants.kPivotLimits.backwardLimit().getRotations(),
                IntakeConstants.PivotConstants.kPivotLimits.forwardLimit().getRotations()
            )
        );
    }

    private void refreshTuneables() {
        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> mIntakePivotIO.setPDConstants(tIntakeKP.get(), tIntakeKD.get()), 
            tIntakeKP, tIntakeKD
        );

        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> setFF(tIntakeKS.get(), tIntakeKG.get(), tIntakeKV.get(), tIntakeKA.get()), 
            tIntakeKS, tIntakeKG, tIntakeKV, tIntakeKA
        );
  
        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> mIntakePivotIO.setMotionMagicConstants(tIntakeCruiseVel.get(), tIntakeMaxAccel.get(), tIntakeMaxJerk.get()), 
            tIntakeCruiseVel, tIntakeMaxAccel, tIntakeMaxJerk
        );
    }
}