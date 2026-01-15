package frc.robot.utils.modifiers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.field.FieldIntersection;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class GroundIntakeAssist implements IDrivetrainControlModifier {

    public enum FailReason {
        NONE,
        NO_OR_UNREACHABLE_CORAL,
        NOT_IN_GROUND_MODE,
        NOT_DRIVING,
        DRIVER_MOVING_IN_OPPOSITE_DIRECTION,
        DRIVER_ROTATE_IN_OPPOSITE_DIRECTION
    }

    public static final double MIN_TARGET_VELOCITY_FOR_ASSIST_CANCEL = 2.0;
    public static final double MIN_DRIVER_VELOCITY_DIFFERENCE_FOR_ASSIST_CANCEL = 0.5;
    public static final double MIN_DRIVER_ROTATION_VELOCITY_DIFFERENCE_FOR_ASSIST_CANCEL = Units.degreesToRadians(40);
    public static final double CORAL_VELOCITY_TANGENCY_THRESHOLD = 0.85;

    public static final double MIN_ASSIST_WEIGHT = 0.2;
    public static final double MAX_ASSIST_WEIGHT = 1.0;
    public static final double WEIGHT_HALF_SCALE_DISTANCE = 3.0;

    private static GroundIntakeAssist defaultInstance;

    private boolean enabled = false;
    private PIDController rotationController = new PIDController(Constants.Assists.GROUND_ASSIST_ROTATION_P, 0.0, 0);

    protected GroundIntakeAssist() {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static synchronized GroundIntakeAssist getDefault() {
        if (defaultInstance == null) {
            defaultInstance =
                    ControlModifierService.getInstance().createModifier(GroundIntakeAssist::new, Priority.ASSIST);
        }
        return defaultInstance;
    }

    @Override
    public boolean apply(DrivetrainControl control) {
        if (!isGroundModeActive()) {
            return false;
        }

        Pose2d robotPose = getRobotPoseWithIntakeOffset();
        Translation2d driverVector = getNormalizedDriverVector(control, robotPose);

        Pose3d closestCoral = findAndLogClosestCoral(robotPose, driverVector);
        if (closestCoral == null) {
            logFailReason(FailReason.NO_OR_UNREACHABLE_CORAL);
            return false;
        }

        return applyAssistControl(control, robotPose, closestCoral);
    }

    @SuppressWarnings("java:S3878")
    private static boolean isGroundModeActive() {
        if (RobotContainer.coralIntake.getState() != CoralIntakeState.GROUND) {
            Logger.recordOutput("GroundIntakeAssist/Corals", new Pose3d[0]);
            Logger.recordOutput("GroundIntakeAssist/TargetCoral", new Pose3d[0]);
            logFailReason(FailReason.NOT_IN_GROUND_MODE);
            return false;
        }
        return true;
    }

    private static Pose2d getRobotPoseWithIntakeOffset() {
        return RobotContainer.poseSensorFusion
                .getEstimatedPosition()
                .transformBy(new Transform2d(Constants.CoralIntake.INTAKE_X_OFFSET, Meters.zero(), Rotation2d.kZero));
    }

    private static Translation2d getNormalizedDriverVector(DrivetrainControl control, Pose2d robotPose) {
        Translation2d driverVector = DrivetrainControl.robotToField(
                        control.getTargetVelocity(), robotPose.getRotation())
                .getTranslation();
        double driverSpeed = driverVector.getNorm();
        return driverVector.div(driverSpeed);
    }

    @SuppressWarnings("java:S3878")
    private static Pose3d findAndLogClosestCoral(Pose2d robotPose, Translation2d driverVector) {
        List<Pose3d> corals = findCoralInDirection(robotPose, driverVector, CORAL_VELOCITY_TANGENCY_THRESHOLD);
        Pose3d closestCoral = findClosestCoral(robotPose, corals);

        Logger.recordOutput("GroundIntakeAssist/Corals", corals.toArray(new Pose3d[0]));
        Logger.recordOutput(
                "GroundIntakeAssist/TargetCoral", closestCoral == null ? new Pose3d[0] : new Pose3d[] {closestCoral});

        return closestCoral;
    }

    private static boolean applyAssistControl(DrivetrainControl control, Pose2d robotPose, Pose3d closestCoral) {
        Translation2d robotToCoral =
                closestCoral.getTranslation().toTranslation2d().minus(robotPose.getTranslation());
        double angle = getAngleFacingIntakeTowardsCoral(robotToCoral);

        Logger.recordOutput(
                "GroundIntakeAssist/TargetPose", new Pose2d(robotPose.getTranslation(), new Rotation2d(angle)));

        Translation2d targetVelocity = clipVelocity(
                DrivetrainControl.fieldToRobot(robotToCoral, robotPose.getRotation())
                        .times(Constants.Assists.GROUND_ASSIST_TRANSLATION_P),
                Constants.Swerve.ROBOT_MAX_SPEED / SimpleMath.SQRT2);

        double angleDiff =
                normalizeAngleDifference(angle - robotPose.getRotation().getRadians());
        Rotation2d targetAngularVelocity = new Rotation2d(angleDiff * Constants.Assists.GROUND_ASSIST_ROTATION_P);

        if (!isDriverMovementCompatible(control, targetVelocity, targetAngularVelocity)) {
            return false;
        }

        double distanceToCoral = robotPose
                .getTranslation()
                .getDistance(closestCoral.getTranslation().toTranslation2d());

        applyAssistVelocity(control, targetVelocity, targetAngularVelocity, distanceToCoral);
        logFailReason(FailReason.NONE);
        return true;
    }

    private static double getAngleFacingIntakeTowardsCoral(Translation2d robotToCoral) {
        return Math.atan2(robotToCoral.getY(), robotToCoral.getX()) - Math.PI / 2;
    }

    private static double normalizeAngleDifference(double angleDiff) {
        if (angleDiff > Math.PI) angleDiff -= SimpleMath.PI2;
        if (angleDiff < -Math.PI) angleDiff += SimpleMath.PI2;
        return angleDiff;
    }

    private static Translation2d clipVelocity(Translation2d velocity, double maxVelocity) {
        double speed = velocity.getNorm();
        if (speed > maxVelocity) {
            return velocity.div(speed).times(maxVelocity);
        } else {
            return velocity;
        }
    }

    private static boolean isDriverMovementCompatible(
            DrivetrainControl control, Translation2d targetVelocity, Rotation2d targetAngularVelocity) {
        double driverX = control.getTargetVelocity().getX();

        if (isDriverMovingOppositeToTarget(driverX, targetVelocity.getX())) {
            logFailReason(FailReason.DRIVER_MOVING_IN_OPPOSITE_DIRECTION);
            return false;
        }

        double driverRot = control.getTargetVelocity().getRotation().getRadians();

        if (isDriverRotatingOppositeToTarget(driverRot, targetAngularVelocity)) {
            logFailReason(FailReason.DRIVER_ROTATE_IN_OPPOSITE_DIRECTION);
            return false;
        }

        return true;
    }

    private static boolean isDriverMovingOppositeToTarget(double driverX, double targetX) {
        return Math.abs(targetX) > MIN_TARGET_VELOCITY_FOR_ASSIST_CANCEL
                && Math.abs(driverX) - Math.abs(targetX) >= MIN_DRIVER_VELOCITY_DIFFERENCE_FOR_ASSIST_CANCEL
                && !SimpleMath.signEq(driverX, targetX);
    }

    private static boolean isDriverRotatingOppositeToTarget(double driverRot, Rotation2d targetAngularVelocity) {
        return Math.abs(driverRot) > 1e-3
                && !SimpleMath.signEq(driverRot, targetAngularVelocity.getRadians())
                && Math.abs(driverRot - targetAngularVelocity.getRadians())
                        > MIN_DRIVER_ROTATION_VELOCITY_DIFFERENCE_FOR_ASSIST_CANCEL;
    }

    private static void applyAssistVelocity(
            DrivetrainControl control,
            Translation2d targetVelocity,
            Rotation2d targetAngularVelocity,
            double distanceToCoral) {

        // Inverse distance weight 0-1
        double t = 1.0 / (distanceToCoral / WEIGHT_HALF_SCALE_DISTANCE + 1.0);

        control.applyWeightedVelocity(
                new Transform2d(
                        targetVelocity.getX(), control.getTargetVelocity().getY(), targetAngularVelocity),
                MathUtil.interpolate(MIN_ASSIST_WEIGHT, MAX_ASSIST_WEIGHT, t));
    }

    private static List<Pose3d> findCoralInDirection(
            Pose2d robotPose, Translation2d direction, double tangencyThreshold) {
        List<Pose3d> corals = new ArrayList<>();

        for (Pose3d coral : RobotContainer.coralDetection.getCorals()) {
            Translation2d robotToCoral =
                    coral.getTranslation().toTranslation2d().minus(robotPose.getTranslation());
            robotToCoral = robotToCoral.div(robotToCoral.getNorm());

            double dot = robotToCoral.getX() * direction.getX() + robotToCoral.getY() * direction.getY();

            if (dot > tangencyThreshold) {
                double angle = getAngleFacingIntakeTowardsCoral(robotToCoral);

                double angleDiff =
                        normalizeAngleDifference(angle - robotPose.getRotation().getRadians());

                boolean reachable = !FieldIntersection.collidesWithField(
                        robotPose.getTranslation(), coral.getTranslation().toTranslation2d());

                // Ignore if the angle is too large or not reachable
                if (reachable && Math.abs(angleDiff) <= Constants.Assists.GROUND_ASSIST_MAX_ANGLE_ERROR.in(Radians)) {
                    corals.add(coral);
                }
            }
        }

        return corals;
    }

    private static Pose3d findClosestCoral(Pose2d robotPose, List<Pose3d> corals) {
        Pose3d closestCoral = null;
        double closestCoralDistance = Double.MAX_VALUE;
        for (Pose3d coral : corals) {
            double distance = robotPose
                    .getTranslation()
                    .getDistance(coral.getTranslation().toTranslation2d());
            if (distance <= Constants.Assists.GROUND_ASSIST_MAX_CORAL_DISTANCE.in(Meters)
                    && distance < closestCoralDistance) {
                closestCoralDistance = distance;
                closestCoral = coral;
            }
        }

        return closestCoral;
    }

    private static void logFailReason(FailReason reason) {
        Logger.recordOutput("GroundIntakeAssist/FailReason", reason);
    }

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
