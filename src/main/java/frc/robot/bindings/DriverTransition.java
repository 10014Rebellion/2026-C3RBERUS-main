package frc.robot.bindings;

import frc.robot.Constants;

public class DriverTransition {
    private final double kSwitchTimeSec = 3;
    private final double kStepAmount = 1.0 / ((int) Math.ceil(kSwitchTimeSec / Constants.kPeriodicSec));
    private double kDefenderFactor = 0.0;
    private DriverType mActiveDriver = DriverType.SCORER;
    private DriverType mDesiredDriver = DriverType.SCORER;

    public enum DriverType {
        SCORER,
        DEFENDER
    } 

    public DriverTransition() {}

    public void switchTo(DriverType pDriverToSwitchTo) {
        if(mActiveDriver != pDriverToSwitchTo) {
            mDesiredDriver = pDriverToSwitchTo;
        }
    }

    public double getScorerFactor() {
        if(mDesiredDriver == DriverType.SCORER) {
            return 1.0 - kDefenderFactor;
        } else { // if (mDesiredDriver == DriverType.DEFENDER) {
            return kDefenderFactor;
        }
    }

    public void periodic() {
        if(mActiveDriver != mDesiredDriver) {
            if(mDesiredDriver == DriverType.DEFENDER) {
                kDefenderFactor = kDefenderFactor + kStepAmount;
                if(kDefenderFactor >= 1) {
                    kDefenderFactor = 1;
                    mActiveDriver = DriverType.DEFENDER;
                }
            }  else { // If desired is scorer
                kDefenderFactor = kDefenderFactor - kStepAmount;

                if(kDefenderFactor <= 0) {
                    kDefenderFactor = 0;
                    mActiveDriver = DriverType.SCORER;
                }
            }
        }
    }
}
