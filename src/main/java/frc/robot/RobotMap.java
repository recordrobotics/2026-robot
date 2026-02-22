package frc.robot;

public final class RobotMap {
    private RobotMap() {}

    public static final class Intake {
        public static final int ARM_LEADER_ID = 16;
        public static final int ARM_FOLLOWER_ID = 17;
        public static final int WHEEL_ID = 18;

        private Intake() {}
    }

    public static final class Turret {
        public static final int MOTOR_ID = 19;

        public static final int FRONT_LEFT_LIMIT_SWITCH_ID = 2; // TODO
        public static final int BACK_LEFT_LIMIT_SWITCH_ID = 3; // TODO
        public static final int BACK_RIGHT_LIMIT_SWITCH_ID = 4; // TODO

        private Turret() {}
    }

    public static final class Shooter {
        public static final int HOOD_ID = 20;
        public static final int FLYWHEEL_LEADER_ID = 21;
        public static final int FLYWHEEL_FOLLOWER_ID = 22;

        private Shooter() {}
    }

    public static final class Spindexer {
        public static final int MOTOR_ID = 14;

        private Spindexer() {}
    }

    public static final class Feeder {
        public static final int MOTOR_ID = 15;

        public static final int BOTTOM_BEAM_BREAK_ID = 0; // TODO
        public static final int TOP_BEAM_BREAK_ID = 1; // TODO

        private Feeder() {}
    }

    public static final class Climber {
        public static final int MOTOR_ID = 23;

        private Climber() {}
    }
}
