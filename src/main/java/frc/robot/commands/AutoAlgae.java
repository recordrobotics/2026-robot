package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.AlgaePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoScore.BackawayAutoControlModifier;
import frc.robot.subsystems.ElevatorHead.AlgaeGrabberStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import frc.robot.utils.AutoUtils;
import frc.robot.utils.modifiers.AutoControlModifier;
import java.util.Set;

public class AutoAlgae extends SequentialCommandGroup {

    private static final double AUTO_LOWER_DISTANCE = 0.3;
    private static final double BACKAWAY_TIMEOUT = 2.0;

    // 8s timeout for first waypoint
    private static final double FIRST_WAYPOINT_TIMEOUT = 8.0;
    // 4s for second
    private static final double OTHER_WAYPOINT_TIMEOUT = 4.0;

    private static boolean cancelCommand = false;
    private static boolean isRunning = false;

    public AutoAlgae(AlgaePosition reefPole) {
        addCommands(
                new InstantCommand(() -> {
                    resetCancel();
                    startRunning();
                }),
                WaypointAlign.alignWithCommand(
                                AlgaeAlign.generateWaypoints(reefPole),
                                new Double[] {FIRST_WAYPOINT_TIMEOUT, OTHER_WAYPOINT_TIMEOUT},
                                // start elevator immediately
                                -1,
                                // elevator has to be fully extended before moving to second waypoint
                                0,
                                algaeGrabCommand(reefPole.getLevel().getHeight())
                                        .asProxy(),
                                AutoControlModifier.getDefault(),
                                AutoUtils::getCurrentDrivetrainKinematicState)
                        .onlyWhile(() -> RobotState.isAutonomous() || !cancelCommand),
                Commands.race(
                        // wait until triggered to go down or manually drove away
                        new WaitUntilCommand(() -> {
                            Pose2d pose = reefPole.getPose();

                            double dist = RobotContainer.poseSensorFusion
                                    .getEstimatedPosition()
                                    .getTranslation()
                                    .getDistance(pose.getTranslation());

                            return cancelCommand || (dist > AUTO_LOWER_DISTANCE);
                        }),
                        // or until algae grabbed (auto backaway)
                        new WaitUntilCommand(() -> RobotContainer.elevatorHead.getGamePiece() == GamePiece.ALGAE)
                                .andThen(Commands.defer(
                                        AutoAlgae::createBackawayCommand, Set.of(RobotContainer.drivetrain)))),
                new ElevatorMoveThenAlgaeGrabEnd(reefPole.getLevel().getHeight(), true).asProxy());
    }

    public static void performCancel() {
        cancelCommand = true;
    }

    public static boolean isRunning() {
        return isRunning;
    }

    public static void stopRunning() {
        isRunning = false;
    }

    private static void resetCancel() {
        cancelCommand = false;
    }

    private static void startRunning() {
        isRunning = true;
    }

    public static Command algaeGrabCommand(ElevatorHeight targetHeight) {
        return new ElevatorMove(
                        targetHeight,
                        AutoScore.ELEVATOR_MOVE_HEIGHT_THRESHOLD,
                        AutoScore.ELEVATOR_MOVE_VELOCITY_THRESHOLD,
                        AutoScore.ELEVATOR_MOVE_ARM_ANGLE_THRESHOLD,
                        AutoScore.ELEVATOR_MOVE_ARM_VELOCITY_THRESHOLD)
                .andThen(new InstantCommand(
                        () -> {
                            if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE)
                                RobotContainer.elevatorHead.set(AlgaeGrabberStates.INTAKE_GROUND);
                            else RobotContainer.elevatorHead.set(AlgaeGrabberStates.INTAKE_REEF);
                        },
                        RobotContainer.elevatorHead));
    }

    public static Command createBackawayCommand() {
        Pose2d startingPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
        Pose2d targetPose = startingPose.transformBy(new Transform2d(-AUTO_LOWER_DISTANCE, 0, Rotation2d.kZero));
        return WaypointAlign.align(
                targetPose,
                BACKAWAY_TIMEOUT,
                BackawayAutoControlModifier.getDefault(),
                AutoUtils::getCurrentDrivetrainKinematicState);
    }
}
