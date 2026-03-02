package frc.robot.logging;

import frc.lib.telemetry.TelemetryConstants.Severity;
import frc.lib.telemetry.TelemetryError;

public class CommandErrors {
    public record NullTriggerSupplier() implements TelemetryError {
        @Override
        public String message() {
            return "NULL BOOLEAN SUPPLIER FOR TRIGGER VALUE";
        }

        @Override
        public Severity severity() {
            return Severity.ERROR;
        }
    }
}
