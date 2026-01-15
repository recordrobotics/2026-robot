package frc.robot;

public final class RobotMap {
    private RobotMap() {}

    public static final class Elevator {
        public static final int MOTOR_LEAD_ID = 10;
        public static final int MOTOR_FOLLOWER_ID = 11;
        public static final int BOTTOM_ENDSTOP_ID = 4;
        public static final int TOP_ENDSTOP_ID = 5;

        private Elevator() {}
    }

    public static final class ElevatorHead {
        public static final int MOTOR_ID = 15;
        public static final int PHOTOSENSOR_ID = 9;

        private ElevatorHead() {}
    }

    public static final class CoralIntake {
        public static final int ARM_ID = 12;
        public static final int WHEEL_ID = 14;

        private CoralIntake() {}
    }

    public static final class ElevatorArm {
        public static final int ARM_ID = 13;

        private ElevatorArm() {}
    }

    public static final class Lights {
        public static final int LED_ID = 0;

        private Lights() {}
    }

    public static final class Climber {
        public static final int MOTOR_ID = 17;
        public static final int RATCHET_SERVO_ID = 1;

        private Climber() {}
    }
}
