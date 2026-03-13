// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;

public class ClimbSS extends SubsystemBase {
    public enum ClimbState {
        UP,
        DOWN,
        STAY,
        STAY_ROBOT,
        IDLE,
        INVALID;
    }
  
    private final ClimbIO mClimbIO;
    private final ClimbInputsAutoLogged mClimbInputs = new ClimbInputsAutoLogged();

    private final ServoIO mServo;

    private ClimbState mClimbState = ClimbState.IDLE;
    private int mDesiredDirection = 0;
    private boolean mLimitEnforced = false;

    public ClimbSS(ClimbIO pClimbIO, ServoIO pServo, PositionSoftLimits pSoftLimits) {
        mClimbIO = pClimbIO;
        mServo = pServo;
    }

    @Override
    public void periodic() {
        mClimbIO.updateInputs(mClimbInputs);
        Logger.processInputs("Climb", mClimbInputs);
        Logger.recordOutput("Climb/LimitsEnforced", mLimitEnforced);
        executeState();
    }

    public void initialState(ClimbState pClimbState) {
        mClimbState = pClimbState;
    }

    public void executeState() {
        switch (mClimbState) {
            case UP, DOWN, STAY, STAY_ROBOT -> {
                mClimbIO.setMotorVolts(ClimbConstants.kStateToVoltage.get(mClimbState).get());
                mServo.setPosition(ClimbConstants.kHookOutPosition);
            }
            case IDLE -> {
                mClimbIO.setMotorVolts(ClimbConstants.kStateToVoltage.get(mClimbState).get());
                mServo.setPosition(ClimbConstants.kHookInPosition);
            } case INVALID -> {}
            default  -> {}
        }
    }

    public Command setStateCmd(ClimbState pState) {
        return new FunctionalCommand(
            () -> initialState(pState), 
            () -> {}, 
            (interrupted) -> {}, 
            () -> false, 
            this);
    }

    public Command goUpTillClimbHeightThenStay() {
        return setStateCmd(ClimbState.UP)
            .until(() -> mClimbInputs.iClimbPositionMeters > ClimbConstants.kClimbHeight)
            .andThen(setStateCmd(ClimbState.STAY));
    }

    public Command goDownTillClimbedThenStayClimbed() {
        return setStateCmd(ClimbState.DOWN)
            .until(() -> mClimbInputs.iClimbPositionMeters < ClimbConstants.kClimbedHeight)
            .andThen(setStateCmd(ClimbState.STAY_ROBOT));
    }

    public void setClimbVolts(double pVolts) {
        mDesiredDirection = toDirection(pVolts);
        mClimbIO.setMotorVolts(pVolts);
        enforceSoftLimits();
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
}
