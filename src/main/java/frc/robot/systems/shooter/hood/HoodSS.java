package frc.robot.systems.shooter.hood;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.shooter.ShotMap;

public class HoodSS extends SubsystemBase {
    // Hood behavior commands replace the previous enum/state-machine.

    private final HoodIO mHoodIO;
    private final ArmFeedforward mHoodFF;
    private final HoodInputsAutoLogged mHoodInputs = new HoodInputsAutoLogged();
  
    private final LoggedTunableNumber tHoodKP = new LoggedTunableNumber("Shooter/Hood/Control/PID/kP", HoodConstants.kHoodControlConfig.pdController().kP());
    private final LoggedTunableNumber tHoodKD = new LoggedTunableNumber("Shooter/Hood/Control/PID/kD", HoodConstants.kHoodControlConfig.pdController().kD());
    private final LoggedTunableNumber tHoodKS = new LoggedTunableNumber("Shooter/Hood/Control/FF/kS", HoodConstants.kHoodControlConfig.feedforward().getKs());
    private final LoggedTunableNumber tHoodKG = new LoggedTunableNumber("Shooter/Hood/Control/FF/kG", HoodConstants.kHoodControlConfig.feedforward().getKg());
    private final LoggedTunableNumber tHoodKV = new LoggedTunableNumber("Shooter/Hood/Control/FF/kV", HoodConstants.kHoodControlConfig.feedforward().getKv());
    private final LoggedTunableNumber tHoodKA = new LoggedTunableNumber("Shooter/Hood/Control/FF/kA", HoodConstants.kHoodControlConfig.feedforward().getKa());
    private final LoggedTunableNumber tHoodCruiseVelDegS = 
        new LoggedTunableNumber("Shooter/Hood/Control/Profile/CruiseVelDegS", HoodConstants.kHoodControlConfig.motionMagicConstants().maxVelocity() * 360);
    private final LoggedTunableNumber tHoodMaxAccelDegS2 = 
        new LoggedTunableNumber("Shooter/Hood/Control/Profile/MaxAccelerationDegs2", HoodConstants.kHoodControlConfig.motionMagicConstants().maxAcceleration() * 360);
    private final LoggedTunableNumber tHoodMaxJerkDegS3 = 
        new LoggedTunableNumber("Shooter/Hood/Control/Profile/MaxJerks3", HoodConstants.kHoodControlConfig.motionMagicConstants().maxJerk() * 360);
    private final LoggedTunableNumber tHoodTolerance = 
        new LoggedTunableNumber("Shooter/Hood/Control/Tolerance", HoodConstants.kTolerance.getDegrees());
  
    // For telemetry compatibility we keep a simple string id for the current behavior.
    @AutoLogOutput(key = "Shooter/Hood/States/CurrentState")
    private String mCurrentHoodBehavior = "STOPPED";

    @AutoLogOutput(key = "Shooter/Hood/LatestClosedLoopGoalRot")
    private Rotation2d mLatestClosedLoopGoalRot = Rotation2d.kZero;
    private int mDesiredDirection = 0;

    @AutoLogOutput(key = "Shooter/Hood/LimitsEnforced")
    private boolean mLimitEnforced = false;
  
    public HoodSS(HoodIO pHoodIO) {
        this.mHoodIO = pHoodIO;
        this.mHoodFF = HoodConstants.kHoodControlConfig.feedforward();
    }

    @Override
    public void periodic() {
        mHoodIO.updateInputs(mHoodInputs);

        refreshTuneables();
        // periodic work is minimal now; behavior commands perform outputs directly.

        Logger.processInputs("Hood", mHoodInputs);
    }

    /*
     * Performs variable updates or parameter intializations when a state is set, SHOULD NOT CHANGE THE STATE THROUGH HERE.
     */
    // Per-behavior command factories. Each returns a Command that performs the
    // same outputs as the old state machine. Commands hold the subsystem while active.

    // Exposed concise factories (public API). These replace the old setStateCmd/HoodStates API.
    public Command stopCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "STOPPED"; mHoodIO.stopMotor(); },
            () -> {},
            this
        );
    }

    public Command tuningVoltageCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "TUNING_VOLTAGE"; setHoodVoltage(HoodConstants.tTuningVoltage.get()); },
            () -> {},
            this
        );
    }

    public Command tuningAmpsCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "TUNING_AMPS"; setHoodAmps(HoodConstants.tTuningAmp.get()); },
            () -> {},
            this
        );
    }

    public Command incrementingCmd() {
        return Commands.runOnce(() -> { mCurrentHoodBehavior = "INCREMENTING"; setHoodPosition(mHoodInputs.iHoodAngle.plus(Rotation2d.fromDegrees(1))); });
    }

    public Command decrementingCmd() {
        return Commands.runOnce(() -> { mCurrentHoodBehavior = "DECREMENTING"; setHoodPosition(mHoodInputs.iHoodAngle.minus(Rotation2d.fromDegrees(1))); });
    }

    public Command holdPositionCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "HOLD_POSITION"; setHoodPosition(mHoodInputs.iHoodAngle); },
            () -> {},
            this
        );
    }

    public Command shotmapPositionCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "SHOTMAP_POSITION"; setHoodPosition(ShotMap.getInstance().getHoodAngle()); },
            () -> {},
            this
        );
    }

    public Command stepIncrementCmd() { return incrementingCmd(); }
    public Command stepDecrementCmd() { return decrementingCmd(); }

    public Command tuningSetpointCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "TUNING_SETPOINT"; setHoodPosition(Rotation2d.fromDegrees(HoodConstants.tTuningShotSetpointDeg.get())); },
            () -> {}, this
        );
    }

    public Command maxCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "MAX"; setHoodPosition(Rotation2d.fromDegrees(HoodConstants.tMaxSetpointDeg.get())); },
            () -> {}, this
        );
    }

    public Command midCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "MID"; setHoodPosition(Rotation2d.fromDegrees(HoodConstants.tMidSetpointDeg.get())); },
            () -> {}, this
        );
    }

    public Command minCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "MIN"; setHoodPosition(Rotation2d.fromDegrees(HoodConstants.tMinSetpointDeg.get())); },
            () -> {}, this
        );
    }

    public Command closeShotCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "CLOSE_SHOT"; setHoodPosition(Rotation2d.fromDegrees(HoodConstants.tCloseShotSetpointDeg.get())); },
            () -> {}, this
        );
    }

    public Command towerShotCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "TOWER_SHOT"; setHoodPosition(Rotation2d.fromDegrees(HoodConstants.tTowerShotSetpointDeg.get())); },
            () -> {}, this
        );
    }

    public Command bumpShotCmd() {
        return Commands.startEnd(
            () -> { mCurrentHoodBehavior = "BUMP_SHOT"; setHoodPosition(Rotation2d.fromDegrees(HoodConstants.tBumpShotSetpointDeg.get())); },
            () -> {}, this
        );
    }

    // Return a lightweight token representing the current behavior to let callers snapshot/restore.
    public String getHoodState() {
        return mCurrentHoodBehavior;
    }

    private void setHoodPosition(Rotation2d pRot) {
        Telemetry.log("Shooter/Hood/Setpoint/NonLimited", pRot);

        pRot = clampRotToSoftLimits(pRot);

        Telemetry.log("Shooter/Hood/Setpoint/Limited", pRot);

        mLatestClosedLoopGoalRot = pRot;

        double ffOutput = mHoodFF.calculate(
            mHoodInputs.iHoodAngle.getRadians(), 
            mHoodInputs.iHoodReferenceValueSlope.getRadians()
        );

        double cos = mHoodInputs.iHoodReferenceValue.getCos();

        Telemetry.log("Shooter/Hood/ffOutput", ffOutput);
        Telemetry.log("Shooter/Hood/ffCos", cos);

        mHoodIO.setMotorPosition(pRot, ffOutput);

        mDesiredDirection = toDirection(getErrorRot());
        enforceSoftLimits();
    }

    private void setHoodVoltage(double pVolts) {
        mDesiredDirection = toDirection(pVolts);
        mHoodIO.setMotorVolts(pVolts);
        enforceSoftLimits();
    }

    private void setHoodAmps(double pAmps) {
        mDesiredDirection = toDirection(pAmps);
        mHoodIO.setMotorAmps(pAmps);
        enforceSoftLimits();
    }

    private void enforceSoftLimits() {
        if(
        (mHoodInputs.iHoodAngle.getRotations() > HoodConstants.kHoodLimits.forwardLimit().getRotations()
            && mDesiredDirection == 1 ) 
            || 
        (mHoodInputs.iHoodAngle.getRotations() < HoodConstants.kHoodLimits.backwardLimit().getRotations() 
            && mDesiredDirection == -1)) {
                mLimitEnforced = true;
                mHoodIO.stopMotor();
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
        mHoodFF.setKs(kS);
        mHoodFF.setKg(kG);
        mHoodFF.setKv(kV);
        mHoodFF.setKa(kA);
    }

    // Restore a previously-captured behavior id by returning the corresponding Command.
    public Command restoreBehaviorCmd(String id) {
        if (id == null) return stopCmd();
        return switch (id) {
            case "STOPPED" -> stopCmd();
            case "TUNING_VOLTAGE" -> tuningVoltageCmd();
            case "TUNING_AMPS" -> tuningAmpsCmd();
            case "INCREMENTING" -> incrementingCmd();
            case "DECREMENTING" -> decrementingCmd();
            case "HOLD_POSITION" -> holdPositionCmd();
            case "SHOTMAP_POSITION" -> shotmapPositionCmd();
            case "STEP_INCREMENT" -> stepIncrementCmd();
            case "STEP_DECREMENT" -> stepDecrementCmd();
            case "TUNING_SETPOINT" -> tuningSetpointCmd();
            case "MAX" -> maxCmd();
            case "MID" -> midCmd();
            case "MIN" -> minCmd();
            case "CLOSE_SHOT" -> closeShotCmd();
            case "TOWER_SHOT" -> towerShotCmd();
            case "BUMP_SHOT" -> bumpShotCmd();
            default -> stopCmd();
        };
    }
  
    @AutoLogOutput(key = "Shooter/Hood/Feedback/ErrorRotation")
    public double getErrorRot() {
        return getCurrentGoal().getRotations() - mHoodInputs.iHoodAngle.getRotations();
    }

    @AutoLogOutput(key = "Shooter/Hood/Feedback/CurrentGoal")
    public Rotation2d getCurrentGoal() {
        return mLatestClosedLoopGoalRot;    
    }

    @AutoLogOutput(key = "Shooter/Hood/Feedback/AtGoal")
    public boolean atGoal() {
        return Math.abs(getErrorRot()) <= tHoodTolerance.get();
    }

    private Rotation2d clampRotToSoftLimits(Rotation2d pRotToClamp) {
        return Rotation2d.fromRotations(
            MathUtil.clamp(
                pRotToClamp.getRotations(),
                HoodConstants.kHoodLimits.backwardLimit().getRotations(),
                HoodConstants.kHoodLimits.forwardLimit().getRotations()
            )
        );
    }

    private void refreshTuneables() {
        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> mHoodIO.setPDConstants(tHoodKP.get(), tHoodKD.get()), 
            tHoodKP, tHoodKD
        );

        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> setFF(tHoodKS.get(), tHoodKG.get(), tHoodKV.get(), tHoodKA.get()), 
            tHoodKS, tHoodKG, tHoodKV, tHoodKA
        );
  

        // Default unit is rotations but the actual values are in degrees. This is to preserve units //
        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> mHoodIO.setMotionMagicConstants(
                tHoodCruiseVelDegS.get() / 360.0, 
                tHoodMaxAccelDegS2.get() / 360.0, 
                tHoodMaxJerkDegS3.get() / 360.0), 
            tHoodCruiseVelDegS, tHoodMaxAccelDegS2, tHoodMaxJerkDegS3
        );
    }
}
