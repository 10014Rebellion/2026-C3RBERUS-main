package frc.lib.controls;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.hardware.HardwareRecords.ArmController;
import frc.lib.hardware.HardwareRecords.ElevatorController;
import frc.lib.hardware.HardwareRecords.SimpleController;

public class SlottedController {

    public enum Mechanism {
        ELEVATOR,
        ARM,
        SIMPLE
    }
    
    public static SimpleController[] mSimpleControllers;
    public static ArmController[] mArmControllers;
    public static ElevatorController[] mElevatorControllers;

    public Mechanism mMechanism;
    

    public SlottedController(Mechanism pMechanism){
        switch(pMechanism) {

            case SIMPLE: 
                mSimpleControllers = new SimpleController[4];
                mArmControllers = null;
                mElevatorControllers = null;
                break;

            case ARM:
                mSimpleControllers = null;
                mArmControllers = new ArmController[4];
                mElevatorControllers = null;
                break;

            case ELEVATOR:
                mSimpleControllers = null;
                mArmControllers = null;
                mElevatorControllers = new ElevatorController[4];
                break;

            default:
                DriverStation.reportError("Tried to call non-existing controller", true);
                 
        }

        this.mMechanism = pMechanism;

    }

    public <T> void setSlot(int slot, T controller) {
        switch (mMechanism) {
            case SIMPLE -> mSimpleControllers[slot] = (SimpleController) controller;
            case ELEVATOR -> mElevatorControllers[slot] = (ElevatorController) controller;
            case ARM -> mArmControllers[slot] = (ArmController) controller;
        }
    }


    public <T> T getSlot(int slot, Class<T> type) {
        Object value = switch (mMechanism) {
            case SIMPLE -> mSimpleControllers[slot];
            case ELEVATOR -> mElevatorControllers[slot];
            case ARM -> mArmControllers[slot];
        };
        return type.cast(value);
    }



    








}
