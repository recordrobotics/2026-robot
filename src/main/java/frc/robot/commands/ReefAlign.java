package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import java.util.List;

public final class ReefAlign {
    private ReefAlign() {}

    public static List<Pose2d> generateWaypointsClosest(boolean includeStartMoveWaypoint) {
        return generateWaypointsClosest(
                DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel(), includeStartMoveWaypoint);
    }

    public static List<Pose2d> generateWaypointsClosestWithOffset(boolean includeStartMoveWaypoint) {
        return generateWaypointsClosestWithOffset(
                DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel(), includeStartMoveWaypoint);
    }

    public static List<Pose2d> generateWaypointsClosest(CoralLevel level, boolean includeStartMoveWaypoint) {
        CoralPosition closestCoral =
                IGamePosition.closestTo(RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values());

        if (closestCoral
                        .getPose(level)
                        .getTranslation()
                        .getDistance(RobotContainer.poseSensorFusion
                                .getEstimatedPosition()
                                .getTranslation())
                > Constants.Align.MAX_REEF_ALIGN_DISTANCE) return List.of();

        return generateWaypoints(closestCoral, level, includeStartMoveWaypoint);
    }

    public static List<Pose2d> generateWaypointsClosestWithOffset(CoralLevel level, boolean includeStartMoveWaypoint) {
        CoralPosition closestCoral =
                IGamePosition.closestTo(RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values());

        if (closestCoral
                        .getPose(level)
                        .getTranslation()
                        .getDistance(RobotContainer.poseSensorFusion
                                .getEstimatedPosition()
                                .getTranslation())
                > Constants.Align.MAX_REEF_ALIGN_DISTANCE) return List.of();

        return generateWaypointsWithOffset(closestCoral, level, includeStartMoveWaypoint);
    }

    public static List<Pose2d> generateWaypoints(
            CoralPosition pole, CoralLevel level, boolean includeStartMoveWaypoint) {
        Transform2d[] transforms = createWaypointsTransforms(level, includeStartMoveWaypoint);
        return WaypointAlign.createWaypointsToTarget(pole.getPose(level), transforms);
    }

    public static List<Pose2d> generateWaypointsWithOffset(
            CoralPosition pole, CoralLevel level, boolean includeStartMoveWaypoint) {
        Transform2d[] transforms = createWaypointsTransforms(level, includeStartMoveWaypoint);
        return WaypointAlign.createWaypointsToTarget(
                pole.getPose(level)
                        .transformBy(new Transform2d(-Constants.Align.ADDITIONAL_OFFSET, 0, Rotation2d.kZero)),
                transforms);
    }

    private static Transform2d[] createWaypointsTransforms(CoralLevel level, boolean includeStartMoveWaypoint) {
        Transform2d[] transforms = new Transform2d[includeStartMoveWaypoint ? 2 : 1];
        if (includeStartMoveWaypoint) {
            transforms[0] = level == CoralLevel.L1
                    ? new Transform2d(0, -Constants.Align.INTAKE_START_MOVE_DISTANCE_L1, Rotation2d.kZero)
                    : new Transform2d(-Constants.Align.ELEVATOR_START_MOVE_DISTANCE, 0, Rotation2d.kZero);
        }
        transforms[includeStartMoveWaypoint ? 1 : 0] = level == CoralLevel.L1
                ? new Transform2d(0, -Constants.Align.INTAKE_END_MOVE_DISTANCE_L1, Rotation2d.kZero)
                : new Transform2d(-Constants.Align.ELEVATOR_END_MOVE_DISTANCE, 0, Rotation2d.kZero);
        return transforms;
    }
}
