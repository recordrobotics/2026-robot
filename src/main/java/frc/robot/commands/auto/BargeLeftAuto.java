package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralShoot;
import frc.robot.utils.AutoUtils;
import frc.robot.utils.CommandUtils;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class BargeLeftAuto extends SequentialCommandGroup implements IAutoRoutine {

    private static final double ALIGN_TIMEOUT = 3.0;
    private static final double SHOOT_TIMEOUT = 1.0;

    public BargeLeftAuto() {
        try {
            addCommands(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeLeftToReefJ")),
                    CommandUtils.finishOnInterrupt(AutoUtils.alignToReefL4().withTimeout(ALIGN_TIMEOUT)),
                    new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                    CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(SHOOT_TIMEOUT)),
                    AutoUtils.createSource("J", "Left"),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefK")),
                    CommandUtils.finishOnInterrupt(AutoUtils.alignToReefL4().withTimeout(ALIGN_TIMEOUT)),
                    new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                    CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(SHOOT_TIMEOUT)),
                    AutoUtils.createSource("K", "Left"),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefL")),
                    CommandUtils.finishOnInterrupt(AutoUtils.alignToReefL4().withTimeout(ALIGN_TIMEOUT)),
                    new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                    CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(SHOOT_TIMEOUT)),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefLToPark")),
                    new InstantCommand(() -> RobotContainer.drivetrain.kill()));
            addRequirements(RobotContainer.drivetrain);
        } catch (FileVersionException | IOException | ParseException e) {
            throw new CreateAutoRoutineException("Failed to create BargeLeftAuto: " + e.getMessage(), e);
        }
    }

    @Override
    public String getAutoName() {
        return "CMD_BargeLeftOuter";
    }
}
