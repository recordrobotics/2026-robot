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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.CoralDetection;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
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
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;

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
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

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

        public enum AlgaeLevel {
            LOW(ElevatorHeight.LOW_REEF_ALGAE),
            HIGH(ElevatorHeight.HIGH_REEF_ALGAE);

            private final ElevatorHeight height;

            private AlgaeLevel(ElevatorHeight height) {
                this.height = height;
            }

            public ElevatorHeight getHeight() {
                return height;
            }
        }

        public static final Distance ALGAE_OFFSET = Meters.of(-0.06);
        public static final Distance ALGAE_REEF_DISTANCE = Centimeters.of(6);

        public enum AlgaePosition implements IGamePosition {
            BLUE_AB(AlgaeLevel.HIGH, 18),
            BLUE_CD(AlgaeLevel.LOW, 17),
            BLUE_EF(AlgaeLevel.HIGH, 22),
            BLUE_GH(AlgaeLevel.LOW, 21),
            BLUE_IJ(AlgaeLevel.HIGH, 20),
            BLUE_KL(AlgaeLevel.LOW, 19),

            RED_AB(AlgaeLevel.HIGH, 7),
            RED_CD(AlgaeLevel.LOW, 8),
            RED_EF(AlgaeLevel.HIGH, 9),
            RED_GH(AlgaeLevel.LOW, 10),
            RED_IJ(AlgaeLevel.HIGH, 11),
            RED_KL(AlgaeLevel.LOW, 6);

            private final AlgaeLevel level;
            private final Pose2d pose;

            private AlgaePosition(AlgaeLevel level, int apriltagId) {
                this.level = level;
                this.pose = calculatePoseFromAprilTag(apriltagId);
            }

            private static Pose2d calculatePoseFromAprilTag(int apriltagId) {
                Optional<Pose3d> tagPose = APRILTAG_LAYOUT.getTagPose(apriltagId);
                if (tagPose.isEmpty()) {
                    throw new IllegalArgumentException("Invalid AprilTag ID: " + apriltagId);
                }

                return Pose2d.toImmutable(tagPose.get()
                        .toPose2d()
                        .transformBy(new Transform2d(
                                Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2 + ALGAE_REEF_DISTANCE.in(Meters),
                                ALGAE_OFFSET.in(Meters),
                                Rotation2d.k180deg)));
            }

            public AlgaeLevel getLevel() {
                return level;
            }

            @Override
            public Pose2d getPose() {
                return pose;
            }
        }

        public enum CoralLevel {
            L1(ElevatorHeight.L1, 1),
            L2(ElevatorHeight.L2, 2),
            L3(ElevatorHeight.L3, 3),
            L4(ElevatorHeight.L4, 4);

            private final ElevatorHeight height;
            private final int level;

            private CoralLevel(ElevatorHeight height, int level) {
                this.height = height;
                this.level = level;
            }

            public ElevatorHeight getHeight() {
                return height;
            }

            public int getLevel() {
                return level;
            }
        }

        public static final Distance REEF_SEGMENT_OFFSET = Meters.of(0.1743095);
        public static final Distance SHOOTER_OFFSET = Meters.of(0.182088);

        public enum CoralPosition implements IGamePosition {
            BLUE_A(18, 0),
            BLUE_B(18, 1),
            BLUE_C(17, 0),
            BLUE_D(17, 1),
            BLUE_E(22, 0),
            BLUE_F(22, 1),
            BLUE_G(21, 0),
            BLUE_H(21, 1),
            BLUE_I(20, 0),
            BLUE_J(20, 1),
            BLUE_K(19, 0),
            BLUE_L(19, 1),

            RED_A(7, 0),
            RED_B(7, 1),
            RED_C(8, 0),
            RED_D(8, 1),
            RED_E(9, 0),
            RED_F(9, 1),
            RED_G(10, 0),
            RED_H(10, 1),
            RED_I(11, 0),
            RED_J(11, 1),
            RED_K(6, 0),
            RED_L(6, 1);

            public static final double FIRST_WAYPOINT_OFFSET = 0.2;
            public static final double L1_REEF_OFFSET = 0.4; // TODO: check during comp - might need to be 0.3
            public static final double L2L3_REEF_OFFSET = 0.02;
            public static final double L4_REEF_OFFSET = 0.05;

            public static final double SPACE_ADDITIONAL_REEF_SEGMENT_OFFSET_LEFT =
                    0.01061; // positive is closer to middle
            public static final double SPACE_ADDITIONAL_REEF_SEGMENT_OFFSET_RIGHT =
                    0.01720; // positive is closer to middle
            public static final double SPACE_ADDITIONAL_REEF_SEGMENT_OFFSET_BACK =
                    -0.011455; // positive is closer to reef

            public static final double COMP_ADDITIONAL_REEF_SEGMENT_OFFSET_LEFT = 0;
            public static final double COMP_ADDITIONAL_REEF_SEGMENT_OFFSET_RIGHT = 0;
            public static final double COMP_ADDITIONAL_REEF_SEGMENT_OFFSET_BACK = 0;

            public static double ADDITIONAL_REEF_SEGMENT_OFFSET_LEFT = SPACE_ADDITIONAL_REEF_SEGMENT_OFFSET_LEFT;
            public static double ADDITIONAL_REEF_SEGMENT_OFFSET_RIGHT = SPACE_ADDITIONAL_REEF_SEGMENT_OFFSET_RIGHT;
            public static double ADDITIONAL_REEF_SEGMENT_OFFSET_BACK = SPACE_ADDITIONAL_REEF_SEGMENT_OFFSET_BACK;

            public final int apriltagId;
            private final int side;
            private final Pose2d pose;

            private CoralPosition(int apriltagId, int side) {
                this.side = side;
                this.pose = calculatePoseFromAprilTag(apriltagId, side);
                this.apriltagId = apriltagId;
            }

            private static Pose2d calculatePoseFromAprilTag(int apriltagId, int side) {
                Optional<Pose3d> tagPose = APRILTAG_LAYOUT.getTagPose(apriltagId);
                if (tagPose.isEmpty()) {
                    throw new IllegalArgumentException("Invalid AprilTag ID: " + apriltagId);
                }

                return Pose2d.toImmutable(tagPose.get()
                        .toPose2d()
                        .transformBy(new Transform2d(
                                Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2,
                                (side == 0 ? -REEF_SEGMENT_OFFSET.in(Meters) : REEF_SEGMENT_OFFSET.in(Meters)),
                                Rotation2d.k180deg)));
            }

            public Pose2d getFirstStagePose() {
                return pose.transformBy(
                        new Transform2d(-FIRST_WAYPOINT_OFFSET, -SHOOTER_OFFSET.in(Meters), Rotation2d.kZero));
            }

            @Override
            public Pose2d getPose() {
                return getFirstStagePose();
            }

            public Pose2d getPose(CoralLevel level) {
                Pose2d newPose = pose.transformBy(new Transform2d(
                        ADDITIONAL_REEF_SEGMENT_OFFSET_BACK,
                        (side == 0 ? -ADDITIONAL_REEF_SEGMENT_OFFSET_LEFT : ADDITIONAL_REEF_SEGMENT_OFFSET_RIGHT),
                        Rotation2d.kZero));
                return switch (level) {
                    case L1 -> newPose.transformBy(new Transform2d(
                            -L1_REEF_OFFSET, CoralIntake.INTAKE_X_OFFSET.in(Meters), Rotation2d.kCW_90deg));
                    case L2, L3 -> newPose.transformBy(
                            new Transform2d(-L2L3_REEF_OFFSET, -SHOOTER_OFFSET.in(Meters), Rotation2d.kZero));
                    case L4 -> newPose.transformBy(
                            new Transform2d(-L4_REEF_OFFSET, -SHOOTER_OFFSET.in(Meters), Rotation2d.kZero));
                    default -> throw new IllegalArgumentException("Invalid Coral Level: " + level);
                };
            }
        }

        public enum SourceCoralSpawnPosition implements IGamePosition {
            // Coral is only used for simulation, no point in making a april tag version
            BLUE_CORAL_LEFT(new Pose2d(0.702, 7.545, Rotation2d.fromDegrees(215.988608))),
            BLUE_CORAL_RIGHT(new Pose2d(0.648, 0.489, Rotation2d.fromDegrees(-35.988608))),
            RED_CORAL_LEFT(BLUE_CORAL_LEFT),
            RED_CORAL_RIGHT(BLUE_CORAL_RIGHT);

            private final Pose2d pose;

            private SourceCoralSpawnPosition(Pose2d pose) {
                this.pose = pose;
            }

            private SourceCoralSpawnPosition(SourceCoralSpawnPosition blueSide) {
                this.pose = Pose2d.toImmutable(FlippingUtil.flipFieldPose(blueSide.getPose()));
            }

            @Override
            public Pose2d getPose() {
                return pose;
            }
        }

        public static final String SOURCE_LEFT_REGION = "Left";
        public static final String SOURCE_RIGHT_REGION = "Right";

        public enum SourcePosition implements IGamePosition {
            BLUE_OUTER_LEFT(13, 1, SOURCE_LEFT_REGION),
            BLUE_OUTER_RIGHT(12, 1, SOURCE_RIGHT_REGION),
            BLUE_INNER_LEFT(13, 0, SOURCE_LEFT_REGION),
            BLUE_INNER_RIGHT(12, 0, SOURCE_RIGHT_REGION),

            RED_OUTER_LEFT(1, 1, SOURCE_LEFT_REGION),
            RED_OUTER_RIGHT(2, 1, SOURCE_RIGHT_REGION),
            RED_INNER_LEFT(1, 0, SOURCE_LEFT_REGION),
            RED_INNER_RIGHT(2, 0, SOURCE_RIGHT_REGION);

            private final Pose2d pose;
            private final String region;

            private SourcePosition(Pose2d pose, String region) {
                this.pose = pose;
                this.region = region;
            }

            private SourcePosition(int apriltagId, int side, String region) {
                this(calculatePoseFromAprilTag(apriltagId, side), region);
            }

            private static Pose2d calculatePoseFromAprilTag(int apriltagId, int side) {
                Optional<Pose3d> tagPose = APRILTAG_LAYOUT.getTagPose(apriltagId);
                if (tagPose.isEmpty()) {
                    throw new IllegalArgumentException("Invalid AprilTag ID: " + apriltagId);
                }

                return Pose2d.toImmutable(tagPose.get()
                        .toPose2d()
                        .transformBy(new Transform2d(
                                Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2,
                                (side == 0
                                        ? -Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2
                                        : Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2),
                                Rotation2d.kCCW_90deg)));
            }

            @Override
            public Pose2d getPose() {
                return pose;
            }

            public String getRegion() {
                return region;
            }
        }

        public enum ProcessorPosition implements IGamePosition {
            BLUE_PROCESSOR(16),
            RED_PROCESSOR(3);

            private final Pose2d pose;

            private ProcessorPosition(int apriltagId) {
                this.pose = calculatePoseFromAprilTag(apriltagId);
            }

            private static Pose2d calculatePoseFromAprilTag(int apriltagId) {
                Optional<Pose3d> tagPose = APRILTAG_LAYOUT.getTagPose(apriltagId);
                if (tagPose.isEmpty()) {
                    throw new IllegalArgumentException("Invalid AprilTag ID: " + apriltagId);
                }

                return Pose2d.toImmutable(tagPose.get()
                        .toPose2d()
                        .transformBy(
                                new Transform2d(Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2, 0, Rotation2d.k180deg)));
            }

            @Override
            public Pose2d getPose() {
                return pose;
            }
        }

        private Game() {}
    }

    public enum FieldStartingLocation {
        BARGE_LEFT(new Pose2d(7.004, 6.171, Rotation2d.fromDegrees(-152.840))),
        BARGE_CENTER(new Pose2d(7.145, 4.026, Rotation2d.fromDegrees(180.000))),
        BARGE_RIGHT(new Pose2d(6.961, 1.939, Rotation2d.fromDegrees(127.847))),
        REEF_TEST(new Pose2d(2.082, 4.053, Rotation2d.fromDegrees(-120))),
        PROCESSOR_TEST(new Pose2d(6.148, 0.606, Rotation2d.fromDegrees(-90)));

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

        public static final double MAX_REEF_ALIGN_DISTANCE = 2.5; // Meters

        public static final double INTAKE_END_MOVE_DISTANCE_L1 = 0.5;
        public static final double INTAKE_START_MOVE_DISTANCE_L1 = 0.65;
        public static final double ELEVATOR_END_MOVE_DISTANCE_L4 = 0.3;
        public static final double ELEVATOR_START_MOVE_DISTANCE_L4 = 1.5;
        public static final double ELEVATOR_END_MOVE_DISTANCE = 0.3;
        public static final double ELEVATOR_START_MOVE_DISTANCE = 1.5;

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

    public static final class Climber {

        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);

        public static final double MAX_ARM_VELOCITY = 10;
        public static final double MAX_ARM_ACCELERATION = 15;
        public static final double MAX_ARM_JERK = 1600;

        public static final double KP = 14.403;
        public static final double KD = 0.1524;
        public static final double KG = 0;
        public static final double KS = 0.02808;
        public static final double KV = 5.4441;
        public static final double KA = 0.1539;

        public static final double CLIMB_VOLTAGE_MAX = 12.0;
        public static final double CLIMB_VOLTAGE_SLEW_RATE =
                CLIMB_VOLTAGE_MAX / 0.3; /* Go from 0 - max in 0.3 seconds */

        public static final double CLIMB_EXPECTED_KV_MIN = 6.0;
        public static final Time CLIMB_EXPECTED_KV_TIMEOUT = Seconds.of(0.9);

        public static final Angle START_ROTATIONS = Rotations.of(0);
        public static final Angle PARK_ROTATIONS = START_ROTATIONS;
        public static final Angle EXTENDED_ROTATIONS = Rotations.of(-1.32);
        public static final Angle CLIMBED_ROTATIONS = Rotations.of(0.15);

        public static final Angle CLIMB_BURST_ROTATIONS = Rotations.of(0.05);

        public static final double RATCHET_ENGAGED = 0.15;
        public static final double RATCHET_DISENGAGED = 0.025;

        public static final double GEAR_RATIO = 48; // 48:1 gearbox

        // TODO: make correct
        public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
        // TODO: make correct
        public static final Distance LENGTH = Meters.of(0.431);
        public static final double ANGLE_OFFSET = 0;

        private Climber() {}
    }

    public static final class ElevatorArm {

        public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(15);
        public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(60);

        public static final double MAX_ARM_VELOCITY = 12;
        public static final double MAX_ARM_ACCELERATION = 40;

        public static final double KP = 43.839;
        public static final double KD = 4.1082;
        public static final double KG = 0.53175;
        public static final double KS = 0.17279;
        public static final double KV = 3.1543;
        public static final double KA = 0.048931;

        public static final double MAX_JERK = 1600;
        public static final double MMEXPO_KV = 1.931;
        public static final double MMEXPO_KA = 1.1;

        public static final double START_POS = Units.degreesToRadians(-90);
        public static final double MIN_POS = Units.degreesToRadians(-103.54);
        public static final double MAX_POS = Units.degreesToRadians(113.79);

        public static final double ARM_GEAR_RATIO = 38.4; // 16:1 * 72/30

        // TODO: make correct
        public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
        // TODO: make correct
        public static final double LENGTH = 0.431;
        public static final double ANGLE_OFFSET = 0;

        public static final Distance MANUAL_CONTROL_MARGIN = Meters.of(0.1);

        private ElevatorArm() {}
    }

    public enum ElevatorHeight {
        INTAKE(Units.inchesToMeters(0.1), Units.degreesToRadians(-103.54)),
        L1(0.26, Units.degreesToRadians(-101.69)),
        L2(0.43, Units.degreesToRadians(-90)),
        LOW_REEF_ALGAE(0.4849, Units.degreesToRadians(-37.841)),
        L3(0.898, Units.degreesToRadians(-101.59)),
        HIGH_REEF_ALGAE(0.916, Units.degreesToRadians(-40.26)),
        L4(1.309, Units.degreesToRadians(54.85)),
        BOTTOM(Constants.Elevator.LOWEST_HOLD_HEIGHT, Units.degreesToRadians(-90)),
        GROUND_ALGAE(Constants.Elevator.LOWEST_HOLD_HEIGHT, Units.degreesToRadians(-36)),
        GROUND_ALGAE_PROCESSOR(0.04, Units.degreesToRadians(5)),
        PROCESSOR_SCORE(0.04, Units.degreesToRadians(-60)),
        BARGE_ALGAE(1.309, Units.degreesToRadians(110));

        private static final double HEIGHT_DIFFERENCE_TOLERANCE = 0.1; // meters
        private static final double ANGLE_DIFFERENCE_TOLERANCE = Units.degreesToRadians(4); // radians

        private final double height;
        private final double armAngleRadians;

        private ElevatorHeight(double heightMeters, double armAngleRadians) {
            this.height = heightMeters;
            this.armAngleRadians = armAngleRadians;
        }

        public double getHeight() {
            return height;
        }

        public double getArmAngle() {
            return armAngleRadians;
        }

        public double getDifference(double height, double angle) {
            return Math.abs(this.height - height) / HEIGHT_DIFFERENCE_TOLERANCE
                    + Math.abs(this.armAngleRadians - angle) / ANGLE_DIFFERENCE_TOLERANCE;
        }
    }

    public static final class Elevator {

        public static final double MAX_VELOCITY = 2.3;
        public static final double MAX_ACCELERATION = 10;
        public static final double KV = 6.8;
        public static final double KA = 0.21006;
        public static final double KG = 0.20027;
        public static final double KS = 0.016248;

        public static final double KP = 68.424;
        public static final double KD = 9.2646;

        public static final double MAX_JERK = 1600;
        public static final double MMEXPO_KV = 5.8;
        public static final double MMEXPO_KA = 1.5;

        public static final double STARTING_HEIGHT = 0;
        public static final double LOWEST_HOLD_HEIGHT = Units.inchesToMeters(1.25);

        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
        public static final Current SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(30);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);

        public static final double GEAR_RATIO = 13.4321 / 2.0 /* compensation for second stage ratio */;
        public static final double DRUM_RADIUS = Units.inchesToMeters(0.8783343);
        // 2 * pi * r / gear ratio because same as getting distance a wheel moved, just
        // vertically
        public static final double METERS_PER_ROTATION = DRUM_RADIUS * 2 * Math.PI / GEAR_RATIO;

        public static final double AT_GOAL_POSITION_TOLERANCE = 0.03;
        public static final double AT_GOAL_VELOCITY_TOLERANCE = 0.63514;

        public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0.15, 0, Rotation2d.fromDegrees(0));
        public static final double MIN_LENGTH = 0.65;
        public static final double MAX_HEIGHT = 1.339;

        public static final Distance MANUAL_CONTROL_MARGIN = Meters.of(0.1);

        private Elevator() {}
    }

    public static final class ElevatorHead {

        public static final double KP = 0.047421;
        public static final double KD = 0.0;
        public static final double KP_POSITION = 6.9;
        public static final double KD_POSITION = 0.132;
        public static final double KS = 0.19168;
        public static final double KV = 3.1111;
        public static final double KA = 0.23988;

        public static final double GEAR_RATIO = 10.0;
        public static final Distance DISTANCE_BETWEEN_AXLES = Inches.of(7);
        public static final Distance CORAL_OUTER_DIAMETER = Inches.of(4.5);
        public static final Distance WHEEL_DIAMETER = DISTANCE_BETWEEN_AXLES.minus(CORAL_OUTER_DIAMETER);

        public static final Distance CORAL_INTAKE_DISTANCE = Inches.of(1.2);

        public static final double SHOOT_STALL_THRESHOLD = 0.2;

        // m/s coral
        public static final double POSITION_MODE_MAX_VELOCITY = 1.4;
        public static final double POSITION_MODE_MAX_ACCELERATION = 0.7;

        public static final double OUT_SPEED_FORWARD = 2.5;
        public static final double OUT_SPEED_BACKWARD = -1.5;
        public static final double INTAKE_SPEED = 0.85;

        public static final double AT_GOAL_POSITION_TOLERANCE = 0.05;
        public static final double AT_GOAL_VELOCITY_TOLERANCE = 0.07;

        public static final Time SHOOT_STALL_TIME = Seconds.of(0.3);

        public static final double DEBOUNCE_TIME = 0.02;
        public static final double DEBOUNCE_TIME_CERTAIN = 0.2;

        public static final double HOW_FAR_FORWARDS_FROM_THE_ELEVATOR_IS_THE_CORAL_SHOOTER = 0.25;

        public static final double OUT_GROUND_SPEED = -8.8;
        public static final double OUT_REEF_SPEED = 2.8;
        public static final double SHOOT_BARGE_SPEED = -16;
        public static final double INTAKE_GROUND_SPEED = 4.2;
        public static final double INTAKE_REEF_SPEED = -5.5;
        public static final double HOLD_GROUND_SPEED = 1.15;
        public static final double HOLD_REEF_SPEED = -5.4;

        public static final double SHOOT_TIME_GROUND = 2.3;
        public static final double SHOOT_TIME_REEF = 0.3;
        public static final double SHOOT_TIME_BARGE = 0.4;

        private ElevatorHead() {}
    }

    public static final class CoralIntake {

        public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(10);
        public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(60);

        public static final double WHEEL_KP = 0.037121;
        public static final double WHEEL_KD = 0.0;
        public static final double WHEEL_KS = 0.1709;
        public static final double WHEEL_KV = 0.61705;
        public static final double WHEEL_KA = 0.039556;

        public static final double MAX_ARM_VELOCITY = 8;
        public static final double MAX_ARM_ACCELERATION = 22;

        public static final double PUSH_OUT_SPEED = 8;
        public static final double PULL_THROUGH_SPEED = 18;
        public static final double SOURCE_SPEED = 18;
        public static final double INTAKE_SPEED = -18;
        public static final double L1_SCORE_SPEED = 20;

        public static final double PUSH_OUT_RAMP_TIME = 0.5;

        public static final double ARM_KP = 68.721;
        public static final double ARM_KD = 8.7176;
        public static final double ARM_KG = 0.47407;
        public static final double ARM_KS = 0.41583;
        public static final double ARM_KV = 5.3;
        public static final double ARM_KA = 0.1;

        public static final double MAX_ARM_JERK = 1600;
        public static final double ARM_MMEXPO_KV = 1.931;
        public static final double ARM_MMEXPO_KA = 1.1;

        public static final double ARM_UP = Units.degreesToRadians(80.19);
        public static final double ARM_PUSH = Units.degreesToRadians(84.19);
        public static final double ARM_INTAKE = Units.degreesToRadians(77.08);
        public static final double ARM_SCORE_L1 = Units.degreesToRadians(22.35);
        public static final double ARM_DOWN = Units.degreesToRadians(-48);
        public static final double ARM_START_POS =
                SysIdManager.getProvider() instanceof frc.robot.subsystems.CoralIntake.SysIdArm
                        ? ARM_DOWN
                        : Units.degreesToRadians(89.3);

        public static final double ARM_GEAR_RATIO = 56.8889; // 16:1 * 64/18

        public static final double WHEEL_GEAR_RATIO = 10;

        public static final double SHOOT_TIME = 0.3;

        public static final Distance INTAKE_X_OFFSET = Inches.of(-3.3999);

        public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
        public static final double LENGTH = 0.385;
        public static final double ANGLE_OFFSET = 0;

        private CoralIntake() {}
    }

    public static final class Lights {

        public static final int LENGTH = 150;

        public static final Dimensionless MULTIPLIER = Percent.of(100);

        public static final Time PULSATE_FREQUENCY = Seconds.of(0.4);
        public static final Time FLASH_FREQUENCY = Seconds.of(0.2);

        public static final double SUCCESS_FLASH_TIME = 1; // seconds

        public static final LinearVelocity SCROLL_SPEED = MetersPerSecond.of(0.3);
        public static final Distance LED_SPACING = Meters.of(1.0 / 30.0); // 30 LEDs per meter

        public static final LEDPattern PULSATING_ORANGE = LEDPattern.solid(Color.kOrange)
                .breathe(Constants.Lights.PULSATE_FREQUENCY)
                .blend(LEDPattern.solid(Color.kOrange));

        public static final LEDPattern PULSATING_GREEN = LEDPattern.solid(Color.kGreen)
                .breathe(Constants.Lights.PULSATE_FREQUENCY)
                .blend(LEDPattern.solid(Color.kGreen));

        public static final LEDPattern FLASHING_GREEN =
                LEDPattern.solid(Color.kGreen).blink(Constants.Lights.FLASH_FREQUENCY);

        public static final LEDPattern CLIMB_PATTERN =
                sparkle(Hertz.of(25), Percent.of(5).per(Second));

        public static final Supplier<LEDPattern> ALLIANCE_COLOR =
                () -> DriverStationUtils.getCurrentAlliance() == Alliance.Red
                        ? LEDPattern.solid(Color.kRed).blend(LEDPattern.solid(Color.kBlack))
                        : LEDPattern.solid(Color.kBlue).blend(LEDPattern.solid(Color.kBlack));

        public static final Supplier<LEDPattern> ALLIANCE_COLOR_FANCY =
                () -> DriverStationUtils.getCurrentAlliance() == Alliance.Red
                        ? LEDPattern.gradient(GradientType.kContinuous, Color.kDarkOrange, Color.kPurple)
                                .mask(LEDPattern.progressMaskLayer(() -> 0.3))
                                .scrollAtAbsoluteSpeed(SCROLL_SPEED.times(5), LED_SPACING)
                                .blend(LEDPattern.solid(Color.kRed))
                        : LEDPattern.gradient(GradientType.kContinuous, Color.kViolet, Color.kPurple)
                                .mask(LEDPattern.progressMaskLayer(() -> 0.3))
                                .scrollAtAbsoluteSpeed(SCROLL_SPEED.times(5), LED_SPACING)
                                .blend(LEDPattern.solid(Color.kBlue));

        public static final Supplier<LEDPattern> ALLIANCE_COLOR_FANCY_WITH_CLIMB =
                () -> RobotContainer.climber.getCurrentState() != ClimberState.PARK
                        ? CLIMB_PATTERN
                        : ALLIANCE_COLOR_FANCY.get();

        public static final LEDPattern elevatorPattern = LEDPattern.solid(Color.kWhite)
                .mask(LEDPattern.progressMaskLayer(
                        () -> RobotContainer.elevator.getCurrentHeight() / Constants.Elevator.MAX_HEIGHT));
        public static final Supplier<LEDPattern> coralIntakePattern = () -> LEDPattern.solid(Color.lerpRGB(
                Color.kRed,
                Color.kGreen,
                (RobotContainer.coralIntake.getArmAngle() - Constants.CoralIntake.ARM_UP)
                        / (Constants.CoralIntake.ARM_DOWN - Constants.CoralIntake.ARM_UP)));
        public static final Supplier<LEDPattern> elevatorHeadPattern =
                () -> (RobotContainer.elevatorHead.getCurrentCoralShooterState() == CoralShooterStates.OFF)
                        ? PULSATING_GREEN
                        : PULSATING_ORANGE;

        public static final Supplier<LEDPattern> sourcePattern =
                () -> LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlack)
                        .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
        public static final Supplier<LEDPattern> reefScorePattern =
                () -> LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kBlack)
                        .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
        public static final Supplier<LEDPattern> algaeScorePattern =
                () -> LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kPurple, Color.kBlack)
                        .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
        public static final Supplier<LEDPattern> cagePattern =
                () -> LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kOrange, Color.kBlack)
                        .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
        public static final Supplier<LEDPattern> removeAlgaePattern =
                () -> LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlack)
                        .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
        public static final Supplier<LEDPattern> coralScorePattern =
                () -> LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kAqua, Color.kBlack)
                        .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
        public static final Supplier<LEDPattern> hybridPattern =
                () -> LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kBlack)
                        .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);

        @SuppressWarnings("java:S6411") // LEDReader as Key is inefficient but acceptable
        private static HashMap<LEDReader, Long> lastSparkles = new HashMap<>();

        private static final Random rand = new Random();

        private Lights() {}

        @SuppressWarnings("java:S109")
        private static LEDPattern sparkle(Frequency frequency, Frequency fadeFrequency) {
            final double periodMicros = frequency.asPeriod().in(Microseconds);
            final double fadePeriodSeconds = fadeFrequency.asPeriod().in(Seconds);
            final double multiplier = MathUtil.clamp(1.0 / fadePeriodSeconds, 0, 1);

            return (reader, writer) -> {
                long now = RobotController.getTime();

                long lastSparkle = lastSparkles.getOrDefault(reader, 0l);

                if (now - lastSparkle > periodMicros) {
                    lastSparkle = now;
                    lastSparkles.put(reader, lastSparkle);

                    int led = rand.nextInt(reader.getLength());
                    writer.setLED(led, Color.kWhite);
                }

                int baseR, baseG, baseB;

                if (DriverStationUtils.getCurrentAlliance() == Alliance.Red) {
                    baseR = 140;
                    baseG = 0;
                    baseB = 0;
                } else {
                    baseR = 0;
                    baseG = 0;
                    baseB = 140;
                }

                for (int led = 0; led < reader.getLength(); led++) {
                    int blendedRGB = Color.lerpRGB(
                            reader.getRed(led),
                            reader.getGreen(led),
                            reader.getBlue(led),
                            baseR,
                            baseG,
                            baseB,
                            multiplier);

                    writer.setRGB(
                            led,
                            Color.unpackRGB(blendedRGB, Color.RGBChannel.kRed),
                            Color.unpackRGB(blendedRGB, Color.RGBChannel.kGreen),
                            Color.unpackRGB(blendedRGB, Color.RGBChannel.kBlue));
                }
            };
        }
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

        public static final double FALCON_DRIVE_KS = 0.12373;
        public static final double FALCON_DRIVE_KV = 2.5609;
        public static final double FALCON_DRIVE_KA = 0.10075;
        public static final double FALCON_DRIVE_KP = 0.1;

        public static final double FALCON_TURN_KV = 1.7518;
        public static final double FALCON_TURN_KA = 0.015791;
        public static final double FALCON_TURN_KS = 0.08889;
        public static final double FALCON_TURN_KP = 1.2;
        public static final double FALCON_TURN_KD = 0.2;

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

        private Swerve() {}

        // Module Creation

        public static ModuleConstants getFrontLeftConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.FRONT_LEFT, DriveMotorType.KRAKEN, TurnMotorType.KRAKEN);
        }

        public static ModuleConstants getFrontRightConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.FRONT_RIGHT, DriveMotorType.KRAKEN, TurnMotorType.KRAKEN);
        }

        public static ModuleConstants getBackLeftConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.BACK_LEFT, DriveMotorType.KRAKEN, TurnMotorType.KRAKEN);
        }

        public static ModuleConstants getBackRightConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.BACK_RIGHT, DriveMotorType.KRAKEN, TurnMotorType.KRAKEN);
        }
    }

    public static final class Auto {

        public static final Time SOURCE_TIMEOUT = Seconds.of(0.8);

        private Auto() {}
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
