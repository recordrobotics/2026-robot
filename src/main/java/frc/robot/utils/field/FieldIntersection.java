package frc.robot.utils.field;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.SimpleMath;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("java:S109")
public final class FieldIntersection {

    private static final FastPolygonIntersection intersection;

    private static final double FIELD_X_MAX = 16.54105;
    private static final double FIELD_Y_MAX = 8.06926;

    private static final double HUB_X_LEN = 1.19380;
    private static final double HUB_Y_LEN = 1.19380;
    private static final double HUB_X = 4.625594;
    private static final double HUB_Y = 4.03463;
    private static final double HUB_RAMP_LENGTH = Inches.of(73.0).in(Meters);

    private static final double UPRIGHT_X_LEN = Inches.of(3.5).in(Meters);
    private static final double UPRIGHT_Y_LEN = Inches.of(1.5).in(Meters);
    private static final double UPRIGHT_OFFSET_FROM_END_WALL = 1.06204;
    private static final double UPRIGHT_OFFSET_FROM_SIDE_WALL = 3.31524;
    private static final double UPRIGHT_Y_SPACING = Inches.of(33.75).in(Meters);

    private static final double TRENCH_WALL_Y_LEN = Inches.of(12.0).in(Meters);
    private static final double TRENCH_WALL_X_LEN = Inches.of(47.0).in(Meters);
    private static final double TRENCH_WALL_OFFSET_FROM_END_WALL = 4.61769;
    private static final double TRENCH_WALL_OFFSET_FROM_SIDE_WALL = 1.43113;

    private static final boolean ADD_RAMP_COLLIDER = true;

    private FieldIntersection() {}

    static {
        // Define field polygons (in meters)
        float[][] polygons = new float[][] {
            // blue tower uprights
            createRectangle(
                    UPRIGHT_X_LEN,
                    UPRIGHT_Y_LEN,
                    new Pose2d(UPRIGHT_OFFSET_FROM_END_WALL, UPRIGHT_OFFSET_FROM_SIDE_WALL, new Rotation2d())),
            createRectangle(
                    UPRIGHT_X_LEN,
                    UPRIGHT_Y_LEN,
                    new Pose2d(
                            UPRIGHT_OFFSET_FROM_END_WALL,
                            UPRIGHT_OFFSET_FROM_SIDE_WALL + UPRIGHT_Y_SPACING,
                            new Rotation2d())),
            // red tower uprights
            createRectangle(
                    UPRIGHT_X_LEN,
                    UPRIGHT_Y_LEN,
                    new Pose2d(
                            FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
                            FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL,
                            new Rotation2d())),
            createRectangle(
                    UPRIGHT_X_LEN,
                    UPRIGHT_Y_LEN,
                    new Pose2d(
                            FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
                            FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL - UPRIGHT_Y_SPACING,
                            new Rotation2d())),
            // blue trench wall
            createRectangle(
                    TRENCH_WALL_X_LEN,
                    TRENCH_WALL_Y_LEN,
                    new Pose2d(TRENCH_WALL_OFFSET_FROM_END_WALL, TRENCH_WALL_OFFSET_FROM_SIDE_WALL, new Rotation2d())),
            createRectangle(
                    TRENCH_WALL_X_LEN,
                    TRENCH_WALL_Y_LEN,
                    new Pose2d(
                            TRENCH_WALL_OFFSET_FROM_END_WALL,
                            FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
                            new Rotation2d())),
            // red trench wall
            createRectangle(
                    TRENCH_WALL_X_LEN,
                    TRENCH_WALL_Y_LEN,
                    new Pose2d(
                            FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
                            TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
                            new Rotation2d())),
            createRectangle(
                    TRENCH_WALL_X_LEN,
                    TRENCH_WALL_Y_LEN,
                    new Pose2d(
                            FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
                            FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
                            new Rotation2d())),
            // blue hub + ramps
            ADD_RAMP_COLLIDER
                    ? createRectangle(
                            HUB_X_LEN, HUB_Y_LEN + 2.0 * HUB_RAMP_LENGTH, new Pose2d(HUB_X, HUB_Y, new Rotation2d()))
                    : createRectangle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(HUB_X, HUB_Y, new Rotation2d())),
            // red hub + ramps
            ADD_RAMP_COLLIDER
                    ? createRectangle(
                            HUB_X_LEN,
                            HUB_Y_LEN + 2.0 * HUB_RAMP_LENGTH,
                            new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()))
                    : createRectangle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()))
        };
        intersection = new FastPolygonIntersection(polygons);
    }

    private static float[] createPolygonFromCorners(Translation2d[] corners) {
        float[] poly = new float[corners.length * 2];
        for (int i = 0; i < corners.length; i++) {
            poly[i * 2] = (float) corners[i].getX();
            poly[i * 2 + 1] = (float) corners[i].getY();
        }
        return poly;
    }

    private static float[] createTriangle(Translation2d a, Translation2d b, Translation2d c) {
        return new float[] {
            (float) a.getX(), (float) a.getY(),
            (float) b.getX(), (float) b.getY(),
            (float) c.getX(), (float) c.getY()
        };
    }

    private static float[] createRectangle(double width, double height, Pose2d pose) {
        Translation2d halfSize = new Translation2d(width / 2, height / 2);
        Translation2d[] corners = new Translation2d[] {
            new Translation2d(-halfSize.getX(), -halfSize.getY())
                    .rotateBy(pose.getRotation())
                    .plus(pose.getTranslation()),
            new Translation2d(halfSize.getX(), -halfSize.getY())
                    .rotateBy(pose.getRotation())
                    .plus(pose.getTranslation()),
            new Translation2d(halfSize.getX(), halfSize.getY())
                    .rotateBy(pose.getRotation())
                    .plus(pose.getTranslation()),
            new Translation2d(-halfSize.getX(), halfSize.getY())
                    .rotateBy(pose.getRotation())
                    .plus(pose.getTranslation())
        };
        return createPolygonFromCorners(corners);
    }

    /**
     * Check if the line segment between points a and b collides with any field obstacles.
     * @param a  The starting point of the line segment.
     * @param b The ending point of the line segment.
     * @return True if the line segment intersects any field obstacles, false otherwise.
     */
    public static boolean collidesWithField(Translation2d a, Translation2d b) {
        if (!SimpleMath.isInField(a) || !SimpleMath.isInField(b)) {
            return true;
        }

        return intersection.intersectsAny((float) a.getX(), (float) a.getY(), (float) b.getX(), (float) b.getY(), true);
    }

    public static void logPolygons() {
        float[][] polys = intersection.getPolygons();
        Translation2d[][] corners = new Translation2d[polys.length][];
        for (int i = 0; i < polys.length; i++) {
            float[] poly = polys[i];
            corners[i] = new Translation2d[poly.length / 2 + 1];
            for (int j = 0; j <= poly.length; j += 2) {
                corners[i][j / 2] = new Translation2d(poly[j % poly.length], poly[(j + 1) % poly.length]);
            }
        }

        Logger.recordOutput(
                "FieldIntersectionCorners",
                Arrays.stream(corners)
                        .flatMap(t -> Arrays.stream(t).map(p -> new Pose2d(p, Rotation2d.kZero)))
                        .toArray(Pose2d[]::new));
    }
}
