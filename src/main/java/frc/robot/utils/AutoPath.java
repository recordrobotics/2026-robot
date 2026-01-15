package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralIntakeMoveL1;
import frc.robot.commands.CoralIntakeShootL1;
import frc.robot.commands.CoralShoot;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.ElevatorMoveThenAlgaeGrabEnd;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.WaypointAlign;
import frc.robot.commands.legacy.CoralIntakeFromSource;
import frc.robot.commands.legacy.ElevatorMoveThenAlgaeGrab;
import frc.robot.utils.libraries.Elastic.Notification.NotificationLevel;
import frc.robot.utils.modifiers.AutoControlModifier;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public final class AutoPath {

    private AutoPath() {}

    @SuppressWarnings("java:S109")
    public static void initialize() {
        RobotConfig config = Constants.Swerve.PP_DEFAULT_CONFIG;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            ConsoleLogger.logError("AutoPath failed to load config", e);
            Notifications.send(NotificationLevel.ERROR, "AutoPath failed to load config", "Error message in console");
        }

        // Registering named commands (so that the pathplanner can call them by name)

        // Stop the robot's movement
        NamedCommands.registerCommand("Stop", new InstantCommand(() -> RobotContainer.drivetrain.kill()));

        NamedCommands.registerCommand(
                "AutoAlign",
                Commands.defer(
                        () -> CommandUtils.finishOnInterrupt(WaypointAlign.align(
                                ReefAlign.generateWaypointsClosestWithOffset(CoralLevel.L4, false),
                                0,
                                1,
                                true,
                                new Double[] {2.0, 2.0},
                                AutoControlModifier.getDefault(),
                                AutoUtils::getCurrentDrivetrainKinematicState)),
                        Set.of(RobotContainer.drivetrain)));
        NamedCommands.registerCommand("ElevatorL4", new ElevatorMove(ElevatorHeight.L4));
        NamedCommands.registerCommand("ElevatorL3", new ElevatorMove(ElevatorHeight.L3));
        NamedCommands.registerCommand("ElevatorL2", new ElevatorMove(ElevatorHeight.L2));
        NamedCommands.registerCommand("ElevatorDown", new ElevatorMove(ElevatorHeight.BOTTOM));
        NamedCommands.registerCommand(
                "AlgaeL2",
                CommandUtils.finishOnInterrupt(ElevatorMoveThenAlgaeGrab.create(ElevatorHeight.LOW_REEF_ALGAE, false)
                        .withTimeout(3.0)));
        NamedCommands.registerCommand(
                "AlgaeL2End",
                CommandUtils.finishOnInterrupt(
                        new ElevatorMoveThenAlgaeGrabEnd(ElevatorHeight.LOW_REEF_ALGAE, false).withTimeout(2.0)));

        NamedCommands.registerCommand(
                "AlgaeL3",
                CommandUtils.finishOnInterrupt(ElevatorMoveThenAlgaeGrab.create(ElevatorHeight.HIGH_REEF_ALGAE, false)
                        .withTimeout(3.0)));
        NamedCommands.registerCommand(
                "AlgaeL3End",
                CommandUtils.finishOnInterrupt(
                        new ElevatorMoveThenAlgaeGrabEnd(ElevatorHeight.HIGH_REEF_ALGAE, false).withTimeout(2.0)));

        NamedCommands.registerCommand(
                "ProcessorScore", CommandUtils.finishOnInterrupt(new ProcessorScore(false).withTimeout(2)));
        NamedCommands.registerCommand(
                "SourceIntake", CommandUtils.finishOnInterrupt(new CoralIntakeFromSource(false).withTimeout(4.5)));
        NamedCommands.registerCommand(
                "SourceIntakeStart", CommandUtils.finishOnInterrupt(new CoralIntakeFromSource(false).withTimeout(4)));
        NamedCommands.registerCommand(
                "SourceIntakeEnd", CommandUtils.finishOnInterrupt(new CoralIntakeFromSource(false).withTimeout(4)));
        NamedCommands.registerCommand("CoralShoot", CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(2)));
        NamedCommands.registerCommand(
                "CoralL1ArmMove", CommandUtils.finishOnInterrupt(new CoralIntakeMoveL1().withTimeout(3)));
        NamedCommands.registerCommand(
                "CoralL1Shoot", CommandUtils.finishOnInterrupt(new CoralIntakeShootL1().withTimeout(3)));

        // Configures auto builder
        AutoBuilder.configure(
                RobotContainer.poseSensorFusion::getEstimatedPosition, // Robot pose supplier
                RobotContainer.poseSensorFusion
                        ::setToPose, // Method to reset odometry (will be called if your auto has a starting
                // pose)
                RobotContainer.drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

                // Method that will drive the robot given ROBOT RELATIVE speeds
                (speeds, feedforwards) -> AutoControlModifier.getDefault().drive(speeds),
                Constants.Swerve.PP_DRIVE_CONTROLLER,
                config,

                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                () -> {
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },

                // Reference to this subsystem to set requirements
                RobotContainer.drivetrain);

        PathPlannerLogging.setLogActivePathCallback(activePath ->
                Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
                targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));
        PathfindingCommand.warmupCommand().schedule();
    }
}
