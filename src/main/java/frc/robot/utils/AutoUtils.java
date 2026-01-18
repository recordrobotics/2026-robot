package frc.robot.utils;

import frc.robot.RobotContainer;
import frc.robot.commands.RuckigAlign;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;

public final class AutoUtils {

    // How long to wait after starting drive to source before deploying intake
    public static final double SOURCE_INTAKE_DEPLOY_DELAY = 0.3;
    // If the robot is within this distance of the closest reef, it will drive to source from there, otherwise go from
    // ElevatorStart waypoint
    public static final double REEF_DISTANCE_THRESHOLD = 0.7;

    private AutoUtils() {}

    public static KinematicState getCurrentDrivetrainKinematicState() {
        return RuckigAlign.toKinematicState(
                RobotContainer.poseSensorFusion.getEstimatedPosition(),
                RobotContainer.drivetrain.getChassisSpeeds(),
                RobotContainer.drivetrain.getChassisAcceleration());
    }
}
