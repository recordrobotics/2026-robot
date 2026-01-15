package frc.robot.utils.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.SimpleMath;
import java.util.Arrays;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("java:S109")
public final class FieldIntersection {

    private static final FastPolygonIntersection intersection;

    private FieldIntersection() {}

    static {
        // blue reef
        Translation2d[] reefVorticesBlue = new Translation2d[] {
            new Translation2d(3.658, 3.546),
            new Translation2d(3.658, 4.506),
            new Translation2d(4.489, 4.987),
            new Translation2d(5.3213, 4.506),
            new Translation2d(5.3213, 3.546),
            new Translation2d(4.489, 3.065)
        };

        // red reef
        Translation2d[] reefVorticesRed = Arrays.stream(reefVorticesBlue)
                .map(pointAtBlue ->
                        new Translation2d(FieldMirroringUtils.FIELD_WIDTH - pointAtBlue.getX(), pointAtBlue.getY()))
                .toArray(Translation2d[]::new);

        // Define field polygons (in meters)
        float[][] polygons = new float[][] {
            // blue coral stations
            createTriangle(new Translation2d(0, 1.270), new Translation2d(1.672, 0), new Translation2d(0, 0)),
            createTriangle(new Translation2d(0, 6.782), new Translation2d(1.672, 8.052), new Translation2d(0, 8.052)),
            // red coral stations
            createTriangle(
                    new Translation2d(17.548, 1.270),
                    new Translation2d(17.548 - 1.672, 0),
                    new Translation2d(17.548, 0)),
            createTriangle(
                    new Translation2d(17.548, 6.782),
                    new Translation2d(17.548 - 1.672, 8.052),
                    new Translation2d(17.548, 8.052)),
            // blue reef
            createPolygonFromCorners(reefVorticesBlue),
            // red reef
            createPolygonFromCorners(reefVorticesRed),
            // the pillar in the middle of the field
            createRectangle(0.305, 0.305, new Pose2d(8.774, 4.026, new Rotation2d()))
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

        Logger.recordOutput("FieldIntersectionCorners", corners);
    }
}
