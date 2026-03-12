package frc.robot.systems.shooter.hood;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class Hood extends SubsystemBase {
    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Hood/Voltage", 0.0);
    public static final LoggedTunableNumber tTuningAmp = new LoggedTunableNumber("Hood/Amp", 0.0);

    public static final LoggedTunableNumber tMaxSetpointDeg  = new LoggedTunableNumber("Hood/MinSetpointDegrees", 0.0);
    public static final Supplier<Rotation2d> kMaxSetpointSup = () -> Rotation2d.fromDegrees(tMaxSetpointDeg.get());

    public static final LoggedTunableNumber tMidSetpointDeg  = new LoggedTunableNumber("Hood/MidSetpointDegrees", 0.0);
    public static final Supplier<Rotation2d> kMidSetpointSup = () -> Rotation2d.fromDegrees(tMidSetpointDeg.get());

    public static final LoggedTunableNumber tMinSetpointDeg  = new LoggedTunableNumber("Hood/MaxSetpointDegrees", 0.0);
    public static final Supplier<Rotation2d> kMinSetpointSup = () -> Rotation2d.fromDegrees(tMinSetpointDeg.get());

    public static final LoggedTunableNumber tCloseShotSetpointDeg  = new LoggedTunableNumber("Hood/CloseShotSetpointDegrees", 0.0);
    public static final Supplier<Rotation2d> kCloseShotSetpointSup = () -> Rotation2d.fromDegrees(tCloseShotSetpointDeg.get());

    public static final LoggedTunableNumber tTowerShotSetpointDeg  = new LoggedTunableNumber("Hood/TowerShotSetpointDegrees", 0.0);
    public static final Supplier<Rotation2d> kTowerShotSetpointSup = () -> Rotation2d.fromDegrees(tTowerShotSetpointDeg.get());

    public static final LoggedTunableNumber tBumpShotSetpointDeg  = new LoggedTunableNumber("Hood/BumpShotSetpointDegrees", 0.0);
    public static final Supplier<Rotation2d> kBumpShotSetpointSup = () -> Rotation2d.fromDegrees(tBumpShotSetpointDeg.get());

    public static final LoggedTunableNumber tTuningShotSetpointDeg  = new LoggedTunableNumber("Hood/TuningShotSetpointDegrees", 0.0);
    public static final Supplier<Rotation2d> kTuningShotSetpointSup = () -> Rotation2d.fromDegrees(tTuningShotSetpointDeg.get());

    public static final LoggedTunableNumber tIncrementSpeedDPS = new LoggedTunableNumber("Hood/IncrementSpeedDPS", 1.0);
    public static final Supplier<Rotation2d> tIncrementSpeedSup = () -> Rotation2d.fromDegrees(tIncrementSpeedDPS.get());

    public static enum HoodStates {
        STOPPED, // At rest
        TUNING_VOLTAGE,
        TUNING_AMPS,
        MAX,
        MID,
        MIN,
        CLOSE_SHOT,
        TOWER_SHOT,
        BUMP_SHOT,
        INCREMENTING,
        DECREMENTING,
        TUNING_SETPOINT
    }

    private final HoodIO mHoodIO;
    private final ArmFeedforward mHoodFF;
    private final HoodInputsAutoLogged mHoodInputs = new HoodInputsAutoLogged();
  
    private final LoggedTunableNumber tHoodKP = new LoggedTunableNumber("Hood/PID/kP", HoodConstants.kHoodControlConfig.pdController().kP());
    private final LoggedTunableNumber tHoodKD = new LoggedTunableNumber("Hood/PID/kD", HoodConstants.kHoodControlConfig.pdController().kD());
    private final LoggedTunableNumber tHoodKS = new LoggedTunableNumber("Hood/FF/kS", HoodConstants.kHoodControlConfig.feedforward().getKs());
    private final LoggedTunableNumber tHoodKG = new LoggedTunableNumber("Hood/FF/kG", HoodConstants.kHoodControlConfig.feedforward().getKg());
    private final LoggedTunableNumber tHoodKV = new LoggedTunableNumber("Hood/FF/kV", HoodConstants.kHoodControlConfig.feedforward().getKv());
    private final LoggedTunableNumber tHoodKA = new LoggedTunableNumber("Hood/FF/kA", HoodConstants.kHoodControlConfig.feedforward().getKa());
    private final LoggedTunableNumber tHoodCruiseVel = new LoggedTunableNumber("Hood/Profile/CruiseVel", HoodConstants.kHoodControlConfig.motionMagicConstants().maxVelocity());
    private final LoggedTunableNumber tHoodMaxAccel = new LoggedTunableNumber("Hood/Profile/MaxAcceleration", HoodConstants.kHoodControlConfig.motionMagicConstants().maxAcceleration());
    private final LoggedTunableNumber tHoodMaxJerk = new LoggedTunableNumber("Hood/Profile/MaxJerk", HoodConstants.kHoodControlConfig.motionMagicConstants().maxJerk());
    private final LoggedTunableNumber tHoodTolerance = new LoggedTunableNumber("Hood/Control/Tolerance", HoodConstants.kToleranceRotations);
  
    @AutoLogOutput(key = "Shooter/Hood/States/CurrentState")
    private HoodStates mCurrentHoodState = HoodStates.STOPPED;

    @AutoLogOutput(key = "Shooter/Hood/RotationGoal/CurrentGoal")
    private Rotation2d oGoalAngle = Rotation2d.kZero;
    private int desiredDirection = 0;


    @AutoLogOutput(key = "Shooter/Hood/LimitsEnforced")
    private boolean limitEnforced = false;
  
    public Hood(HoodIO pHoodIO) {
        this.mHoodIO = pHoodIO;
        this.mHoodFF = HoodConstants.kHoodControlConfig.feedforward();
    }

    @Override
    public void periodic() {
        mHoodIO.updateInputs(mHoodInputs);

        refreshTuneables();
        runActionBasedOnState();

        Logger.processInputs("Hood", mHoodInputs);
    }

    private void runActionBasedOnState() {
        switch (mCurrentHoodState) {
              case STOPPED -> {
                mHoodIO.stopMotor();
            } case TUNING_VOLTAGE -> {
                setHoodVoltage(tTuningVoltage.get());
            } case TUNING_AMPS -> {
                setHoodAmps(tTuningAmp.get());
            } case MAX -> {
                setHoodPosition(kMaxSetpointSup.get());
            } case MID -> {
                setHoodPosition(kMidSetpointSup.get());
            } case MIN -> {
                setHoodPosition(kMinSetpointSup.get());
            } case CLOSE_SHOT -> {
                setHoodPosition(kCloseShotSetpointSup.get());
            } case BUMP_SHOT -> {
                setHoodPosition(kBumpShotSetpointSup.get());
            } case TOWER_SHOT -> {
                setHoodPosition(kTowerShotSetpointSup.get());
            } case INCREMENTING -> {
                setHoodPosition(mHoodInputs.iHoodAngle.plus(Rotation2d.fromDegrees(tIncrementSpeedDPS.get() * 0.02)));
            } case DECREMENTING -> {
                setHoodPosition(mHoodInputs.iHoodAngle.minus(Rotation2d.fromDegrees(tIncrementSpeedDPS.get() * 0.02)));
            } case TUNING_SETPOINT -> {
                setHoodPosition(kTuningShotSetpointSup.get());
            }
        }
    }

    public Command incrementOnceCommand() {
        return Commands.runOnce(() -> incrementAngle = 
            mHoodInputs.iHoodAngle.plus(HoodConstants.kIncrementStepAmount), this)
        .andThen(setStateCommandContinued(HoodStates.INCREMENT_ONCE));
    }

    public Command setStateCommandContinued(HoodStates pNewState) {
        return setStateCommandContinued(pNewState, true);
    }

    public Command setStateCommandContinued(HoodStates pNewState, boolean holdRequirementContinuously) {
        return new FunctionalCommand(
            () -> setState(pNewState), 
            () -> {}, 
            (interrupted) ->  {}, 
            () -> !holdRequirementContinuously, 
            this);
    }

    private void setState(HoodStates pNewState) {
        this.mCurrentHoodState = pNewState;
    }

    public void setHoodPosition(Rotation2d pRot) {
        Logger.recordOutput("IntakePivot/Setpoint/Non-limited", pRot);

        pRot = clampRotToSoftLimits(pRot);

        Logger.recordOutput("IntakePivot/Setpoint/Limited", pRot);

        oGoalAngle = pRot;

        double ffOutput = mHoodFF.calculate(
            mHoodInputs.iHoodAngle.getRadians(), 
            mHoodInputs.iHoodReferenceValue.getRadians()
        );

        double cos = mHoodInputs.iHoodReferenceValue.getCos();

        Logger.recordOutput("Intake/Pivot/ffOutput", ffOutput);
        Logger.recordOutput("Intake/Pivot/cos", cos);

        mHoodIO.setMotorPosition(pRot, ffOutput);

        desiredDirection = toDirection(getErrorPositionRotations());
        enforceSoftLimits();
    }

    public void setHoodVoltage(double pVolts) {
        desiredDirection = toDirection(pVolts);
        mHoodIO.setMotorVolts(pVolts);
        enforceSoftLimits();
    }

    public void setHoodAmps(double pAmps) {
        desiredDirection = toDirection(pAmps);
        mHoodIO.setMotorAmps(pAmps);
        enforceSoftLimits();

    }

    public void enforceSoftLimits() {
        if(
        (mHoodInputs.iHoodAngle.getRotations() > HoodConstants.kHoodLimits.forwardLimit().getRotations()
            && desiredDirection == 1 ) 
            || 
        (mHoodInputs.iHoodAngle.getRotations() < HoodConstants.kHoodLimits.backwardLimit().getRotations() 
            && desiredDirection == -1)) {
                limitEnforced = true;
                mHoodIO.stopMotor();
        } else {
            limitEnforced = false;
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

    public HoodStates getHoodState() {
        return mCurrentHoodState;
    }
  
    @AutoLogOutput(key = "Shooter/Hood/Feedback/ErrorRotation")
    public double getErrorPositionRotations() {
        return getCurrentGoal().getRotations() - mHoodInputs.iHoodAngle.getRotations();
    }

    @AutoLogOutput(key = "Shooter/Hood/Feedback/CurrentGoal")
    public Rotation2d getCurrentGoal() {
        return oGoalAngle;    
    }

    @AutoLogOutput(key = "Shooter/Hood/Feedback/AtGoal")
    public boolean atGoal() {
        return Math.abs(getErrorPositionRotations()) < tHoodTolerance.get();
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
  
        LoggedTunableNumber.ifChanged( hashCode(), 
            () -> mHoodIO.setMotionMagicConstants(tHoodCruiseVel.get(), tHoodMaxAccel.get(), tHoodMaxJerk.get()), 
            tHoodCruiseVel, tHoodMaxAccel, tHoodMaxJerk
        );
    }
}
