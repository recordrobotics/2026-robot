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

public class BargeRightAuto extends SequentialCommandGroup implements IAutoRoutine {

    private static final double ALIGN_TIMEOUT = 3.0;
    private static final double SHOOT_TIMEOUT = 1.0;

    public BargeRightAuto() {
        try {
            addCommands(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeRightToReefE")),
                    CommandUtils.finishOnInterrupt(AutoUtils.alignToReefL4().withTimeout(ALIGN_TIMEOUT)),
                    new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                    CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(SHOOT_TIMEOUT)),
                    AutoUtils.createSource("E", "Right"),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefD")),
                    CommandUtils.finishOnInterrupt(AutoUtils.alignToReefL4().withTimeout(ALIGN_TIMEOUT)),
                    new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                    CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(SHOOT_TIMEOUT)),
                    AutoUtils.createSource("D", "Right"),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefC")),
                    CommandUtils.finishOnInterrupt(AutoUtils.alignToReefL4().withTimeout(ALIGN_TIMEOUT)),
                    new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                    CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(SHOOT_TIMEOUT)),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefCToPark")),
                    new InstantCommand(() -> RobotContainer.drivetrain.kill()));
            addRequirements(RobotContainer.drivetrain);
        } catch (FileVersionException | IOException | ParseException e) {
            throw new CreateAutoRoutineException("Failed to create BargeRightAuto: " + e.getMessage(), e);
        }
    }

    @Override
    public String getAutoName() {
        return "CMD_BargeRightOuter";
    }
}
