package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.WeakHashMap;

public interface PositionedSubsystem {

    enum PositionStatus {
        /**
         * The position is unknown, possible at the start position
         */
        UNKNOWN,
        /**
         * The position is accurate and known
         */
        KNOWN,
        /**
         * The position could not be determined due to a mechanical fault.
         * (For example elevator hit physical stop above limit switch)
         */
        MECHANICAL_FAULT,
        /**
         * The position could not be determined due to a sensor fault.
         * (For example encoder or sensor disconnected)
         */
        SENSOR_FAULT
    }

    void resetToStartPosition();

    PositionStatus getPositionStatus();

    void setOverrideKnown(boolean overrideKnown);

    class PositionedSubsystemManager {

        private static PositionedSubsystemManager instance;

        private record Entry(Alert warningAlert, Alert errorAlert, String name) {

            private void setWarning(String errorMessage) {
                warningAlert.setText(errorMessage);
                warningAlert.set(true);
                errorAlert.set(false);
            }

            private void setError(String errorMessage) {
                errorAlert.setText(errorMessage);
                warningAlert.set(false);
                errorAlert.set(true);
            }

            private void clear() {
                warningAlert.set(false);
                errorAlert.set(false);
            }

            public String overridePath() {
                return "Positioned/" + name + "/OverrideKnown";
            }
        }

        private final WeakHashMap<PositionedSubsystem, Entry> subsystems = new WeakHashMap<>();

        public static PositionedSubsystemManager getInstance() {
            if (instance == null) {
                instance = new PositionedSubsystemManager();
            }
            return instance;
        }

        public void registerSubsystem(PositionedSubsystem subsystem) {
            Entry entry = new Entry(
                    new Alert("", AlertType.kWarning),
                    new Alert("", AlertType.kError),
                    subsystem.getClass().getSimpleName());
            subsystems.put(subsystem, entry);
            SmartDashboard.putBoolean(entry.overridePath(), false);
        }

        private static void updateEntry(PositionedSubsystem subsystem, Entry entry) {
            boolean overrideKnown = SmartDashboard.getBoolean(entry.overridePath(), false);
            subsystem.setOverrideKnown(overrideKnown);

            if (overrideKnown) {
                entry.setWarning(entry.name() + " override known");
            } else {
                PositionStatus status = subsystem.getPositionStatus();
                switch (status) {
                    case UNKNOWN -> entry.setWarning(entry.name() + " unknown position");
                    case KNOWN -> entry.clear();
                    case MECHANICAL_FAULT -> entry.setError(entry.name() + " mechanical fault");
                    case SENSOR_FAULT -> entry.setError(entry.name() + " sensor fault");
                }
            }
        }

        public void update() {
            subsystems.forEach(PositionedSubsystemManager::updateEntry);
        }

        public void resetAll() {
            subsystems.keySet().forEach(PositionedSubsystem::resetToStartPosition);
        }
    }
}
