package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.Game.AlgaePosition;
import java.util.List;

public final class AlgaeAlign {

    private static final double APPROACH_DISTANCE = 0.3;

    private AlgaeAlign() {}

    public static List<Pose2d> generateWaypoints(AlgaePosition pole) {
        return WaypointAlign.createWaypointsToTarget(
                pole.getPose(), new Transform2d[] {new Transform2d(-APPROACH_DISTANCE, 0, Rotation2d.kZero)});
    }
}
