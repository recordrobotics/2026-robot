package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.ElevatorArm;
import frc.robot.utils.AutoUtils;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.AutoControlModifier;
import frc.robot.utils.modifiers.ControlModifierService;
import frc.robot.utils.modifiers.DrivetrainControl;
import java.util.Set;

public class AutoScore extends SequentialCommandGroup {

    public static final double ELEVATOR_MOVE_HEIGHT_THRESHOLD = Constants.Elevator.AT_GOAL_POSITION_TOLERANCE + 1.17;
    public static final double ELEVATOR_MOVE_VELOCITY_THRESHOLD = Constants.Elevator.AT_GOAL_VELOCITY_TOLERANCE + 999;
    public static final double ELEVATOR_MOVE_ARM_ANGLE_THRESHOLD = ElevatorArm.POSITION_TOLERANCE + 1.94;
    public static final double ELEVATOR_MOVE_ARM_VELOCITY_THRESHOLD = ElevatorArm.VELOCITY_TOLERANCE + 999;

    // 8s timeout for first waypoint
    private static final double FIRST_WAYPOINT_TIMEOUT = 8.0;
    // 4s for second and third
    private static final double OTHER_WAYPOINT_TIMEOUT = 4.0;

    private static final double BACKAWAY_DISTANCE = 0.6;
    private static final double BACKAWAY_TIMEOUT = 2.0;

    private static final double L4_MIN_DISTANCE_FROM_REEF_TO_LOWER = 0.3;

    @SuppressWarnings("java:S3358") // there is enough clarity for nested ternary operators
    public AutoScore(CoralPosition reefPole, CoralLevel level) {
        addCommands(
                Commands.defer(
                        () -> {
                            Pose2d robotPose = RobotContainer.poseSensorFusion.getEstimatedPosition();

                            boolean insideElevatorMoveRegion = robotPose
                                            .getTranslation()
                                            .getDistance(reefPole.getPose(level).getTranslation())
                                    < Constants.Align.ELEVATOR_START_MOVE_DISTANCE;

                            return WaypointAlign.alignWithCommand(
                                    ReefAlign.generateWaypoints(reefPole, level, !insideElevatorMoveRegion),
                                    new Double[] {FIRST_WAYPOINT_TIMEOUT, OTHER_WAYPOINT_TIMEOUT, OTHER_WAYPOINT_TIMEOUT
                                    },
                                    // start elevator immediately if already inside move region, otherwise only after
                                    // first waypoint
                                    insideElevatorMoveRegion ? -1 : 0,
                                    // elevator has to be fully extended before moving to second/third waypoint
                                    insideElevatorMoveRegion ? 0 : 1,
                                    (level != CoralLevel.L1
                                                    ? new ElevatorMove(
                                                            level.getHeight(),
                                                            ELEVATOR_MOVE_HEIGHT_THRESHOLD,
                                                            ELEVATOR_MOVE_VELOCITY_THRESHOLD,
                                                            ELEVATOR_MOVE_ARM_ANGLE_THRESHOLD,
                                                            ELEVATOR_MOVE_ARM_VELOCITY_THRESHOLD)
                                                    : new CoralIntakeMoveL1())
                                            .asProxy(),
                                    AutoControlModifier.getDefault(),
                                    AutoUtils::getCurrentDrivetrainKinematicState);
                        },
                        Set.of(RobotContainer.drivetrain)),
                // if ruckig timed out, wait until autoscore is pressed again
                new WaitUntilCommand(() -> DashboardUI.Overview.getControl().isAutoScoreTriggered())
                        .onlyIf(() -> !RuckigAlign.lastAlignSuccessful() && !RobotState.isAutonomous()),
                // wait until trigger not pressed (only in teleop) AND elevator/arm at goal before shooting (not in L1)
                new WaitUntilCommand(() -> (RobotState.isAutonomous()
                                        || !DashboardUI.Overview.getControl().isAutoScoreTriggered())
                                && (level == CoralLevel.L1
                                        || (RobotContainer.elevator.atGoal() && RobotContainer.elevatorArm.atGoal())))
                        .andThen(
                                level != CoralLevel.L1
                                        ? new CoralShoot()
                                                .andThen(Commands.defer(
                                                        AutoScore::createBackawayCommand,
                                                        Set.of(RobotContainer.drivetrain)))
                                        : new CoralIntakeShootL1().asProxy()));
    }

    private static Command createBackawayCommand() {
        Pose2d startingPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
        Pose2d targetPose = startingPose.transformBy(new Transform2d(-BACKAWAY_DISTANCE, 0, Rotation2d.kZero));
        return WaypointAlign.align(
                        targetPose,
                        BACKAWAY_TIMEOUT,
                        BackawayAutoControlModifier.getDefault(),
                        AutoUtils::getCurrentDrivetrainKinematicState)
                .onlyWhile(() -> RobotContainer.poseSensorFusion
                                .getEstimatedPosition()
                                .getTranslation()
                                .getDistance(startingPose.getTranslation())
                        <= BACKAWAY_DISTANCE) // cancel if manually drove far enough away
                .alongWith(new WaitUntilCommand(() -> RobotContainer.poseSensorFusion
                                        .getEstimatedPosition()
                                        .getTranslation()
                                        .getDistance(startingPose.getTranslation())
                                >= (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.L4
                                        ? L4_MIN_DISTANCE_FROM_REEF_TO_LOWER
                                        : 0))
                        .andThen(new ElevatorMove(ElevatorHeight.BOTTOM).asProxy())
                        .onlyIf(() -> !CoralShoot.failedToShoot()));
    }

    @SuppressWarnings("java:S6548") // Singleton for default instance
    public static final class BackawayAutoControlModifier extends AutoControlModifier {
        private static BackawayAutoControlModifier defaultInstance;

        private BackawayAutoControlModifier() {}

        @SuppressWarnings("EffectivelyPrivate")
        public static synchronized BackawayAutoControlModifier getDefault() {
            if (defaultInstance == null) {
                defaultInstance = ControlModifierService.getInstance()
                        .createModifier(BackawayAutoControlModifier::new, Priority.AUTO);
            }
            return defaultInstance;
        }

        @Override
        protected boolean applyChassisSpeeds(ChassisSpeeds speeds, DrivetrainControl control) {
            double backawaySpeed = speeds.vxMetersPerSecond;
            double targetSpeed = control.getTargetVelocity().getX();

            // Driver input override backaway only if in same direction and greater
            if (SimpleMath.signEq(targetSpeed, backawaySpeed) && Math.abs(targetSpeed) > Math.abs(backawaySpeed)) {
                backawaySpeed = targetSpeed;
            }

            control.applyWeightedVelocity(
                    new Transform2d(
                            backawaySpeed,
                            control.getTargetVelocity().getY(),
                            control.getTargetVelocity().getRotation()),
                    1);
            return true;
        }
    }
}
