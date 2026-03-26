// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSS extends SubsystemBase {
    // Behavior IDs (string keys) used for telemetry and restore semantics
    private final ClimbIO mClimbIO;
    private final ClimbInputsAutoLogged mClimbInputs = new ClimbInputsAutoLogged();

    private final AngularServoIO mServo;

    @AutoLogOutput(key = "Climb/State")
    private String mCurrentClimbBehavior = "IDLE";

    private int mDesiredDirection = 0;
    private boolean mLimitEnforced = false;

    public ClimbSS(ClimbIO pClimbIO, AngularServoIO pServo) {
        mClimbIO = pClimbIO;
        mServo = pServo;
    }

    @Override
    public void periodic() {
        mClimbIO.updateInputs(mClimbInputs);
        Logger.processInputs("Climb", mClimbInputs);
        Logger.recordOutput("Climb/LimitsEnforced", mLimitEnforced);
        // mCurrentClimbBehavior is AutoLogged above
        executeState();
    }

    public void initiateState(String pClimbState) {
        mCurrentClimbBehavior = pClimbState;
    }

    public void executeState() {
        switch (mCurrentClimbBehavior) {
            case "UP", "DOWN", "STAY", "STAY_ROBOT" -> {
                setClimbVolts(ClimbConstants.kStateToVoltage.get(mCurrentClimbBehavior).get());
                mServo.setPosition(ClimbConstants.kHookOutPosition);
            }
            case "IDLE" -> {
                setClimbVolts(ClimbConstants.kStateToVoltage.get(mCurrentClimbBehavior).get());
                mServo.setPosition(ClimbConstants.kHookInPosition);
            }
            default  -> {}
        }
    }

    /* Generic replacement for the old setStateCmd — accepts a string behavior id. */
    public Command setStateCmd(String pNewBehavior) { return setStateCmd(pNewBehavior, true); }

    public Command setStateCmd(String pNewBehavior, boolean holdRequirementContinuously) {
        if (holdRequirementContinuously) {
            return Commands.startEnd(() -> setState(pNewBehavior), () -> {}, this);
        } else {
            return Commands.runOnce(() -> setState(pNewBehavior));
        }
    }

    public Command goUpTillClimbHeightThenStay() {
        return setStateCmd("UP")
            .until(() -> mClimbInputs.iClimbPositionMeters > ClimbConstants.kClimbHeight)
            .andThen(setStateCmd("STAY"));
    }

    public Command goDownTillClimbedThenStayClimbed() {
        return setStateCmd("DOWN")
            .until(() -> mClimbInputs.iClimbPositionMeters < ClimbConstants.kClimbedHeight)
            .andThen(setStateCmd("STAY_ROBOT"));
    }

    public void setClimbVolts(double pVolts) {
        mDesiredDirection = toDirection(pVolts);
        mClimbIO.setMotorVolts(pVolts);
        enforceSoftLimits();
    }

    public void changeClimbNeutralMode(NeutralModeValue pNeutralMode){
        mClimbIO.changeClimbNeutralMode(pNeutralMode);
    }

    public boolean atGoal(double pGoal) {
        return Math.abs(pGoal - mClimbInputs.iClimbPositionMeters) < 0.01;
    }

    public void enforceSoftLimits() {
        if(
        (mClimbInputs.iClimbPositionMeters > ClimbConstants.kSoftLimits.forwardLimitM()
            && mDesiredDirection == 1 ) 
            || 
        (mClimbInputs.iClimbPositionMeters < ClimbConstants.kSoftLimits.backwardLimitM() 
            && mDesiredDirection == -1)) {
                mLimitEnforced = true;
                mClimbIO.stopMotor();
        } else {
            mLimitEnforced = false;
        }
    }

    public int toDirection(double val) {
        if(val > 0) return 1;
        if(val < 0) return -1;
        else return 0;
    }

    /* Generic state setter used by the command factories */
    private void setState(String pNewBehavior) {
        // no-op end/initialize hooks for now, but keep for parity with other SS
        endState(mCurrentClimbBehavior);
        mCurrentClimbBehavior = pNewBehavior;
        initializeState(pNewBehavior);
    }

    @SuppressWarnings("unused")
    private void initializeState(String pBehaviorToInit) { }

    @SuppressWarnings("unused")
    private void endState(String pBehaviorToEnd) { }

    /* Convenience command factories for Climb states */
    public Command upCmd() { return setStateCmd("UP"); }
    public Command downCmd() { return setStateCmd("DOWN"); }
    public Command stayCmd() { return setStateCmd("STAY"); }
    public Command stayRobotCmd() { return setStateCmd("STAY_ROBOT"); }
    public Command idleCmd() { return setStateCmd("IDLE"); }
}
