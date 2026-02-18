// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.CoralDetection;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.DriveMotorType;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.TurnMotorType;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.wrappers.ImmutableCurrent;
import frc.robot.utils.wrappers.Pose2d;
import frc.robot.utils.wrappers.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {}

    public final class Game {

        public static final AprilTagFieldLayout APRILTAG_LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public interface IGamePosition {
            Pose2d getPose();

            static <E extends IGamePosition> E closestTo(edu.wpi.first.math.geometry.Pose2d pose, E[] values) {
                E closest = null;
                double closestDistance = Double.MAX_VALUE;
                for (E pos : values) {
                    double distance = pos.getPose().getTranslation().getDistance(pose.getTranslation());
                    if (distance < closestDistance) {
                        closest = pos;
                        closestDistance = distance;
                    }
                }
                return closest;
            }

            static Pose2d[] aggregatePositions(IGamePosition[]... values) {
                List<Pose2d> poses = new ArrayList<>();
                for (IGamePosition[] value : values) {
                    for (IGamePosition pos : value) {
                        poses.add(pos.getPose());
                    }
                }
                return poses.toArray(new Pose2d[0]);
            }
        }

        private Game() {}
    }

    public enum FieldStartingLocation {
        DEFAULT(new Pose2d(1, 1, Rotation2d.kZero)); // TODO remove

        private final Pose2d transformRed;
        private final Pose2d transformBlue;

        private FieldStartingLocation(Pose2d poseBlue) {
            transformRed = Pose2d.toImmutable(FlippingUtil.flipFieldPose(poseBlue));
            transformBlue = poseBlue;
        }

        public Pose2d getPose() {
            return DriverStationUtils.getCurrentAlliance() == Alliance.Red ? transformRed : transformBlue;
        }
    }

    public static final class Align {

        public static final double MAX_VELOCITY = Constants.Swerve.ROBOT_MAX_SPEED; // m/s
        public static final double MAX_ANGULAR_VELOCITY = 1.8; // rad/s
        public static final double MAX_ACCELERATION = 3.3228; // m/s^2
        public static final double MAX_DECELERATION = 11.0; // m/s^2
        public static final double MAX_JERK = 2 * 7.0711; // m/s^3
        public static final double MAX_DEJERK = 2 * 7.0711; // m/s^3

        public static final double TRANSLATIONAL_TOLERANCE = 0.01; // Meters
        public static final double TRANSLATIONAL_VELOCITY_TOLERANCE = 0.05; // Meters/s
        public static final double ROTATIONAL_TOLERANCE = Math.toRadians(1); // Radians
        public static final double ROTATIONAL_VELOCITY_TOLERANCE = Math.toRadians(5); // Radians/s

        public static final double ADDITIONAL_OFFSET = 0.02;

        private Align() {}
    }

    public static final class PhotonVision {

        public static final String PHOTON_L1_NAME = "photon-l1";
        public static final String PHOTON_SOURCE_NAME = "photon-source";
        public static final String PHOTON_CORAL_INTAKE = "coral-intake";

        public static final Transform3d ROBOT_TO_CAMERA_L1 = new Transform3d(
                new Translation3d(-0.031750, 0.373120, 0.196097),
                new Rotation3d(0, Units.degreesToRadians(-12.894), Units.degreesToRadians(90)));
        public static final Transform3d ROBOT_TO_CAMERA_SOURCE = new Transform3d(
                new Translation3d(-0.006350, -0.370769, 0.197020),
                new Rotation3d(0, Units.degreesToRadians(-18.951), Units.degreesToRadians(-90)));
        public static final Transform3d ROBOT_TO_CAMERA_GROUND_INTAKE = new Transform3d(
                new Translation3d(Meters.of(0.142850), Meters.of(0.369435), Meters.of(0.526114)),
                new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(90.0 + 25.0)));

        public static final Distance CORAL_ID_DISTANCE = Inches.of(8);
        public static final Time CORAL_TIMEOUT = Seconds.of(0.5);

        public static final int CORAL_ID = 1;

        private PhotonVision() {}
    }

    public static final class Limelight {

        public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
        public static final String LIMELIGHT_CENTER_NAME = "limelight-center";

        public static final Transform3d ROBOT_TO_CAMERA_LEFT = new Transform3d(
                new Translation3d(0.311558, 0.330204, 0.246383), new Rotation3d(0, Units.degreesToRadians(-21), 0));
        public static final Transform3d ROBOT_TO_CAMERA_CENTER = new Transform3d(
                new Translation3d(0.219412, -0.050800, 0.156247), new Rotation3d(0, Units.degreesToRadians(-27), 0));

        private Limelight() {}
    }

    public static final class Assists {

        public static final Distance GROUND_ASSIST_MAX_CORAL_DISTANCE = Meters.of(4);
        public static final Angle GROUND_ASSIST_MAX_ANGLE_ERROR = Degrees.of(60);
        public static final double GROUND_ASSIST_ROTATION_P = 5.0;
        public static final double GROUND_ASSIST_TRANSLATION_P = 5.0;

        private Assists() {}
    }

    public static final class Control {

        // Sensitivity for speed meter
        public static final double DIRECTIONAL_SPEED_METER_LOW = 0.25;
        public static final double DIRECTIONAL_SPEED_METER_HIGH = 4.7;
        public static final double SPIN_SPEED_METER_LOW = 0.5;
        public static final double SPIN_SPEED_METER_HIGH = 2.4;

        // Sensitivies for directional controls (XY) and spin (theta)
        public static final double JOYSTICK_DIRECTIONAL_SENSITIVITY = 1;
        public static final double JOYSTICK_SPIN_SENSITIVITY = 2;
        public static final double JOYSTICK_X_THRESHOLD = 0.25;
        public static final double JOYSTICK_Y_THRESHOLD = 0.25;
        public static final double JOYSTICK_SPIN_THRESHOLD = 0.3;

        // Thresholds for directional controls (XY) and spin (theta)
        public static final double XBOX_DIRECTIONAL_SENSITIVITY = 1;
        public static final double XBOX_X_THRESHOLD = 0.15;
        public static final double XBOX_Y_THRESHOLD = 0.15;
        public static final double XBOX_SPIN_THRESHOLD = 0.3;

        public static final double XBOX_SPIN_ROT_THRESHOLD = 0.1;
        public static final double XBOX_SPIN_ROT_SENSITIVITY = 1.0;

        // How many seconds ahead of robot position does autoscore/autoalgae use to find target
        public static final double SCORE_TARGET_LOOKAHEAD = 0.35;

        private Control() {}
    }

    public static final class Frame {

        /**
         * Distance between wheels (width aka between left and right and length aka between front and
         * back). Used for calculating wheel locations on the robot
         */
        public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.5588;

        public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.5588;

        public static final double FRAME_WIDTH = Inches.of(30).in(Meters);
        public static final double FRAME_WITH_BUMPER_WIDTH =
                FRAME_WIDTH + Inches.of(6.5).in(Meters);
        public static final double MAX_MECHANISM_HEIGHT = 2.1336;

        public static final double ROBOT_MASS = 64.864; // kg
        public static final double ROBOT_MOI = 14.547; // kg*m^2

        private Frame() {}
    }

    public static final class Swerve {

        public static final double PERIODIC = 0.02;

        // Works out module locations
        public static final double WHEEL_EXTENT_X = Frame.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
        public static final double WHEEL_EXTENT_Y = Frame.ROBOT_WHEEL_DISTANCE_LENGTH / 2;
        // Distance from center of robot to wheel
        public static final double WHEEL_BASE_RADIUS = Math.hypot(WHEEL_EXTENT_X, WHEEL_EXTENT_Y);

        public static final Translation2d FRONT_LEFT_WHEEL_LOCATION = new Translation2d(WHEEL_EXTENT_X, WHEEL_EXTENT_Y);
        public static final Translation2d FRONT_RIGHT_WHEEL_LOCATION =
                new Translation2d(WHEEL_EXTENT_X, -WHEEL_EXTENT_Y);
        public static final Translation2d BACK_LEFT_WHEEL_LOCATION = new Translation2d(-WHEEL_EXTENT_X, WHEEL_EXTENT_Y);
        public static final Translation2d BACK_RIGHT_WHEEL_LOCATION =
                new Translation2d(-WHEEL_EXTENT_X, -WHEEL_EXTENT_Y);

        // Gear ratios for falcon and kraken
        public static final double FALCON_TURN_GEAR_RATIO =
                15.43; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)
        public static final double FALCON_DRIVE_GEAR_RATIO =
                7.36; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)

        public static final double KRAKEN_TURN_GEAR_RATIO = 13.3714;
        public static final double KRAKEN_DRIVE_GEAR_RATIO = 6.75; // X1 12 pinion

        public static final ImmutableCurrent FALCON_TURN_STATOR_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(100));
        public static final ImmutableCurrent FALCON_TURN_SUPPLY_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(25));
        public static final ImmutableCurrent FALCON_DRIVE_STATOR_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(120));
        public static final ImmutableCurrent FALCON_DRIVE_SUPPLY_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(32));

        public static final ImmutableCurrent KRAKEN_TURN_STATOR_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(100));
        public static final ImmutableCurrent KRAKEN_TURN_SUPPLY_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(25));
        public static final ImmutableCurrent KRAKEN_DRIVE_STATOR_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(120));
        public static final ImmutableCurrent KRAKEN_DRIVE_SUPPLY_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(52));

        public static final double FALCON_DRIVE_KS = 0.13192;
        public static final double FALCON_DRIVE_KV = 2.7547;
        public static final double FALCON_DRIVE_KA = 0.24758;
        public static final double FALCON_DRIVE_KP = 4.6957;

        public static final double FALCON_TURN_KV = 1.7214;
        public static final double FALCON_TURN_KA = 0.049311;
        public static final double FALCON_TURN_KS = 0.13609;
        public static final double FALCON_TURN_KP = 67.02;
        public static final double FALCON_TURN_KD = 3.2831;

        public static final double KRAKEN_DRIVE_KS = 0.13192;
        public static final double KRAKEN_DRIVE_KV = 2.7547;
        public static final double KRAKEN_DRIVE_KA = 0.24758;
        public static final double KRAKEN_DRIVE_KP = 4.6957;

        public static final double KRAKEN_TURN_KV = 1.2993;
        public static final double KRAKEN_TURN_KA = 0.058972;
        public static final double KRAKEN_TURN_KS = 0.51562;
        public static final double KRAKEN_TURN_KP = 55.543;
        public static final double KRAKEN_TURN_KD = 2.3952;

        // Wheel diameter
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        // Turn max velocity and acceleration
        // Calculated from motor rpm 5000 / 60 (rps) / gear ratio (15.43)
        public static final double TURN_MAX_ANGULAR_VELOCITY = 5; // ROTATIONS / SECOND
        // Calculated from max velocity / time to reach (0.1)
        public static final double TURN_MAX_ANGULAR_ACCELERATION = 20; // ROTATIONS / SECOND / SECOND

        public static final double DRIVE_MAX_ACCELERATION = 13.18;
        public static final double DRIVE_MAX_JERK = 131.28;

        public static final double TURN_MAX_JERK = 1600;
        public static final double TURN_MMEXPO_KV = 1.5;
        public static final double TURN_MMEXPO_KA = 0.02;

        /** The max speed the robot can travel safely */
        public static final double ROBOT_MAX_SPEED = 4.35;

        public static final RobotConfig PP_DEFAULT_CONFIG = new RobotConfig(
                Constants.Frame.ROBOT_MASS,
                Constants.Frame.ROBOT_MOI,
                new ModuleConfig(
                        WHEEL_DIAMETER / 2,
                        ROBOT_MAX_SPEED,
                        1.2,
                        DCMotor.getKrakenX60(1),
                        Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO,
                        52,
                        1),
                FRONT_LEFT_WHEEL_LOCATION,
                FRONT_RIGHT_WHEEL_LOCATION,
                BACK_LEFT_WHEEL_LOCATION,
                BACK_RIGHT_WHEEL_LOCATION);

        public static final PPHolonomicDriveController PP_DRIVE_CONTROLLER = new PPHolonomicDriveController(
                new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.5, 0.0, 0.0) // Rotation PID constants
                );

        public static final DriveMotorType driveMotorType = DriveMotorType.KRAKEN;
        public static final TurnMotorType turnMotorType = TurnMotorType.KRAKEN;

        private Swerve() {}

        // Module Creation

        public static ModuleConstants getFrontLeftConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.FRONT_LEFT, driveMotorType, turnMotorType);
        }

        public static ModuleConstants getFrontRightConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.FRONT_RIGHT, driveMotorType, turnMotorType);
        }

        public static ModuleConstants getBackLeftConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.BACK_LEFT, driveMotorType, turnMotorType);
        }

        public static ModuleConstants getBackRightConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.BACK_RIGHT, driveMotorType, turnMotorType);
        }
    }

    public static final class Auto {

        public static final Time SOURCE_TIMEOUT = Seconds.of(0.8);

        private Auto() {}
    }

    public static final class Intake {
        public static final int MAX_INTAKE_CAPACITY =
                8; // number of fuel that can remain above intake without falling into hopper with intake extended
        // TODO
        public static final double EJECT_FUEL_PER_SECOND = 10.0;

        public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(10);
        public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(60);
        public static final Current WHEEL_SUPPLY_CURRENT_LIMIT = Amps.of(10);
        public static final Current WHEEL_STATOR_CURRENT_LIMIT = Amps.of(60);

        public static final double ARM_MMEXPO_KV = 1.931;
        public static final double ARM_MMEXPO_KA = 1.1;

        public static final double ARM_KP = 68.721;
        public static final double ARM_KD = 8.7176;
        public static final double ARM_KG = 0.47407;
        public static final double ARM_KS = 0.41583;
        public static final double ARM_KV = 5.3;
        public static final double ARM_KA = 0.1;

        public static final double WHEEL_KP = 0.037121;
        public static final double WHEEL_KD = 0.0;
        public static final double WHEEL_KS = 0.1709;
        public static final double WHEEL_KV = 0.61705;
        public static final double WHEEL_KA = 0.039556;

        public static final double ARM_DOWN_POSITION_RADIANS = Units.degreesToRadians(
                0.0); // should always be 0, TODO offset motionmagic gravity arm position offset based on cg
        public static final double ARM_RETRACTED_POSITION_RADIANS = Units.degreesToRadians(81.0);
        public static final double ARM_STARTING_POSITION_RADIANS = Units.degreesToRadians(120.0);

        public static final double WHEEL_INTAKE_VELOCITY_MPS =
                Constants.Swerve.ROBOT_MAX_SPEED * 2; // surface speed of roller // TODO make correct
        public static final double WHEEL_EJECT_VELOCITY_MPS = -6.0; // surface speed of roller // TODO make correct

        public static final double ARM_GEAR_RATIO = 56.8889;
        public static final double WHEEL_GEAR_RATIO = 10;

        public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
        public static final double LENGTH = 0.385;
        public static final double ANGLE_OFFSET = 0;

        private Intake() {}
    }

    public static final class Hopper {
        public static final int MAX_ROBOT_CAPACITY =
                50; // number of fuel that can fit in the whole robot with intake extended
        public static final int MAX_HOPPER_CAPACITY =
                MAX_ROBOT_CAPACITY - Intake.MAX_INTAKE_CAPACITY; // number of fuel that can fit in the hopper alone

        private Hopper() {}
    }

    public final class RobotState {

        public static final boolean MOTOR_LOGGING_ENABLED = false;

        public static final AutoLogLevel.Level AUTO_LOG_LEVEL = getAutoLogLevel();

        public static final VisionSimulationMode VISION_SIMULATION_MODE = VisionSimulationMode.PHOTON_SIM_INACCURATE;

        // change to use an external photonvision client for coral detection simulation
        public static final CoralDetection.CoralDetectionSimulationMode CORAL_DETECTION_SIMULATION_MODE =
                CoralDetection.CoralDetectionSimulationMode.MAPLE_SIM;

        /**
         * <p>
         * Enable NT and Advantage Scope for unit tests.
         * </p>
         * <p>
         * WARNING! ONLY ENABLE THIS IF RUNNING A SINGLE UNIT TEST
         * </p>
         * <p>
         * RUNNING TESTS IN PARALLEL IS NOT SUPPORTED
         * </p>
         *
         * Example: {@code ./gradlew test --tests "*ReefAutoScoreTests`$Blue4"}
         */
        public static final boolean UNIT_TESTS_ENABLE_ADVANTAGE_SCOPE = false;

        private static boolean runningAsUnitTest = false;

        private RobotState() {}

        public static void setRunningAsUnitTest() {
            runningAsUnitTest = true;
        }

        public static Mode getMode() {
            if (RobotBase.isReal()) return Mode.REAL;
            if (runningAsUnitTest) return Mode.TEST;
            return RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY;
        }

        private static AutoLogLevel.Level getAutoLogLevel() {
            if (RobotBase.isReal()) {
                return SysIdManager.getProvider().isEnabled() ? AutoLogLevel.Level.SYSID : AutoLogLevel.Level.REAL;
            } else {
                return AutoLogLevel.Level.SIM;
            }
        }

        public enum VisionSimulationMode {
            /**
             * Uses PhotonVision simulation with accurate april tag placement on the field
             */
            PHOTON_SIM_ACCURATE(true),
            /**
             * Uses PhotonVision simulation with simulated inaccurate april tag placement on the field
             */
            PHOTON_SIM_INACCURATE(true),
            /**
             * Uses the maplesim actual robot pose as a vision measurement
             */
            MAPLE_CLEAN(false),
            /**
             * Uses the maplesim actual robot pose with noise added as a vision measurement
             */
            MAPLE_NOISE(false);

            final boolean isPhotonSim;

            VisionSimulationMode(boolean isPhotonSim) {
                this.isPhotonSim = isPhotonSim;
            }

            public boolean isPhotonSim() {
                return isPhotonSim;
            }
        }

        public enum Mode {
            REAL(true),
            SIM(true),
            REPLAY(false),
            TEST(false);

            final boolean realtime;

            Mode(boolean realtime) {
                this.realtime = realtime;
            }

            public boolean isRealtime() {
                return realtime;
            }
        }
    }
}
