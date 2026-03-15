package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.utils.libraries.Elastic.NotificationLevel;
import frc.robot.utils.modifiers.AutoControlModifier;
import java.util.Optional;
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
        NamedCommands.registerCommand(
                "Intake",
                Commands.run(() -> RobotContainer.intake.setState(IntakeState.INTAKE), RobotContainer.intake));
        NamedCommands.registerCommand(
                "IntakeDepot",
                Commands.run(() -> RobotContainer.intake.setState(IntakeState.INTAKE), RobotContainer.intake)
                        .withTimeout(2.0));
        NamedCommands.registerCommand(
                "Mixer",
                Commands.repeatingSequence(
                        Commands.runOnce(
                                () -> RobotContainer.intake.setState(IntakeState.INTAKE), RobotContainer.intake),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(
                                () -> RobotContainer.intake.setState(IntakeState.RETRACTED), RobotContainer.intake),
                        Commands.waitSeconds(0.5)));

        // Configures auto builder
        AutoBuilder.configure(
                RobotContainer.poseSensorFusion::getEstimatedPosition, // Robot pose supplier
                RobotContainer.poseSensorFusion
                        ::setToPose, // Method to reset odometry (will be called if your auto has a starting
                // pose)
                RobotContainer.drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

                // Method that will drive the robot given ROBOT RELATIVE speeds
                (speeds, feedforwards) -> AutoControlModifier.getDefault()
                        .drive(
                                speeds,
                                feedforwards.robotRelativeForcesXNewtons(),
                                feedforwards.robotRelativeForcesYNewtons()),
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
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }
}
