package frc.robot.commands.auto;

public class Auto {

    public static class AutoLoadException extends RuntimeException {
        public AutoLoadException(String message, Throwable cause) {
            super(message, cause);
        }
    }
}
