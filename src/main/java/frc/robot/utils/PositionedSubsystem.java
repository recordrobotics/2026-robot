package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
        }

        private final WeakHashMap<PositionedSubsystem, Entry> subsystems = new WeakHashMap<>();

        public static PositionedSubsystemManager getInstance() {
            if (instance == null) {
                instance = new PositionedSubsystemManager();
            }
            return instance;
        }

        public void registerSubsystem(PositionedSubsystem subsystem) {
            subsystems.put(
                    subsystem,
                    new Entry(
                            new Alert("", AlertType.kWarning),
                            new Alert("", AlertType.kError),
                            subsystem.getClass().getSimpleName()));
        }

        private static void updateAlert(PositionStatus status, Entry entry) {
            switch (status) {
                case UNKNOWN -> entry.setWarning(entry.name() + " unknown position");
                case KNOWN -> entry.clear();
                case MECHANICAL_FAULT -> entry.setError(entry.name() + " mechanical fault");
                case SENSOR_FAULT -> entry.setError(entry.name() + " sensor fault");
            }
        }

        public void update() {
            subsystems.forEach((subsystem, entry) -> updateAlert(subsystem.getPositionStatus(), entry));
        }

        public void resetAll() {
            subsystems.keySet().forEach(PositionedSubsystem::resetToStartPosition);
        }
    }
}
