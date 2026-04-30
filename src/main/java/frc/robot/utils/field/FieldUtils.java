package frc.robot.utils.field;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.utils.DriverStationUtils;

public final class FieldUtils {

    private static final double TRENCH_WIDTH_METERS = 1.361281;
    private static final double TRENCH_OFFSET_METERS = 0.5;

    private static final double ALLIANCE_ZONE_THRESHOLD_X = 4.0; // TODO make correct

    private static final Translation2d BLUE_TRENCH_HP_SIDE = new Translation2d(4.622, 0.644493);
    private static final Translation2d BLUE_TRENCH_DEPOT_SIDE =
            new Translation2d(BLUE_TRENCH_HP_SIDE.getX(), FlippingUtil.fieldSizeY - BLUE_TRENCH_HP_SIDE.getY());
    private static final Translation2d RED_TRENCH_HP_SIDE = new Translation2d(
            FlippingUtil.fieldSizeX - BLUE_TRENCH_HP_SIDE.getX(), FlippingUtil.fieldSizeY - BLUE_TRENCH_HP_SIDE.getY());
    private static final Translation2d RED_TRENCH_DEPOT_SIDE = new Translation2d(
            FlippingUtil.fieldSizeX - BLUE_TRENCH_DEPOT_SIDE.getX(),
            FlippingUtil.fieldSizeY - BLUE_TRENCH_DEPOT_SIDE.getY());

    private FieldUtils() {}

    public static boolean isInTrench(Translation2d start, Translation2d end, Translation2d trench) {
        double minX = trench.getX() - TRENCH_OFFSET_METERS;
        double maxX = trench.getX() + TRENCH_OFFSET_METERS;
        double minY = trench.getY() - TRENCH_WIDTH_METERS / 2.0;
        double maxY = trench.getY() + TRENCH_WIDTH_METERS / 2.0;

        boolean startInside =
                start.getX() >= minX && start.getX() <= maxX && start.getY() >= minY && start.getY() <= maxY;
        boolean endInside = end.getX() >= minX && end.getX() <= maxX && end.getY() >= minY && end.getY() <= maxY;
        if (startInside || endInside) {
            return true;
        }

        // Slab intersection (segment param t in [0, 1])
        double tMin = 0.0;
        double tMax = 1.0;
        final double eps = 1e-9;

        // X slab
        double dx = end.getX() - start.getX();
        if (Math.abs(dx) < eps) {
            if (start.getX() < minX || start.getX() > maxX) {
                return false;
            }
        } else {
            double tx1 = (minX - start.getX()) / dx;
            double tx2 = (maxX - start.getX()) / dx;
            if (tx1 > tx2) {
                double temp = tx1;
                tx1 = tx2;
                tx2 = temp;
            }
            tMin = Math.max(tMin, tx1);
            tMax = Math.min(tMax, tx2);
            if (tMin > tMax) {
                return false;
            }
        }

        // Y slab
        double dy = end.getY() - start.getY();
        if (Math.abs(dy) < eps) {
            if (start.getY() < minY || start.getY() > maxY) {
                return false;
            }
        } else {
            double ty1 = (minY - start.getY()) / dy;
            double ty2 = (maxY - start.getY()) / dy;
            if (ty1 > ty2) {
                double temp = ty1;
                ty1 = ty2;
                ty2 = temp;
            }
            tMin = Math.max(tMin, ty1);
            tMax = Math.min(tMax, ty2);
            if (tMin > tMax) {
                return false;
            }
        }

        return tMax >= 0.0 && tMin <= 1.0;
    }

    public static boolean isInTrench(Translation2d start, Translation2d end) {
        return isInTrench(start, end, BLUE_TRENCH_HP_SIDE)
                || isInTrench(start, end, BLUE_TRENCH_DEPOT_SIDE)
                || isInTrench(start, end, RED_TRENCH_HP_SIDE)
                || isInTrench(start, end, RED_TRENCH_DEPOT_SIDE);
    }

    public static boolean isInAllianceZone(Translation2d translation) {
        return DriverStationUtils.getCurrentAlliance() == Alliance.Blue
                ? translation.getX() < ALLIANCE_ZONE_THRESHOLD_X
                : translation.getX() > FlippingUtil.fieldSizeX - ALLIANCE_ZONE_THRESHOLD_X;
    }

    public static boolean isInAllianceZone() {
        return isInAllianceZone(
                RobotContainer.poseSensorFusion.getEstimatedPosition().getTranslation());
    }
}
