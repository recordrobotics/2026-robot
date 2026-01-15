package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralShoot;
import frc.robot.utils.AutoUtils;
import frc.robot.utils.CommandUtils;
import java.io.IOException;
import java.util.List;
import java.util.stream.Stream;
import org.json.simple.parser.ParseException;

public class Auto {

    private static final double ALIGN_TIMEOUT = 2.0;
    private static final double SHOOT_TIMEOUT = 1.0;

    public record ScoringPosition(CoralPosition position, CoralLevel level) {}

    public record PPScoringPath(String branch, int level) {}

    public Command create(
            List<ScoringPosition> scoringPositions, FieldStartingLocation startPosition, SourcePosition source)
            throws FileVersionException, IOException, ParseException {
        return create(
                scoringPositions.stream()
                        .map(p -> new PPScoringPath(
                                String.valueOf(p.position()
                                        .name()
                                        .charAt(p.position().name().length() - 1)),
                                p.level().getLevel()))
                        .toList(),
                startPosition,
                source.getRegion());
    }

    /**
     * Create the autonomous command sequence to score on the given reef branches and levels,
     * starting from the given position.
     * @param scoringPaths List of reef branches and levels to score on, in order
     * @param startPosition FieldStartingLocation
     * @param source "Left" or "Right"
     * @return the command
     * @throws FileVersionException If the path files are of an incorrect version
     * @throws IOException If there is an error in loading the path files
     * @throws ParseException If there is an error in reading the path files
     */
    public Command create(List<PPScoringPath> scoringPaths, FieldStartingLocation startPosition, String source)
            throws FileVersionException, IOException, ParseException {

        Command firstPosition = createFirstPositionCommand(scoringPaths.get(0), startPosition, source);
        Command[] subsequentPositions = createSubsequentPositionCommands(scoringPaths, source);

        Command cmd = Commands.sequence(Stream.concat(Stream.of(firstPosition), Stream.of(subsequentPositions))
                .toArray(Command[]::new));
        cmd.addRequirements(RobotContainer.drivetrain);
        return cmd;
    }

    private static Command createFirstPositionCommand(
            PPScoringPath path, FieldStartingLocation startPosition, String source)
            throws FileVersionException, IOException, ParseException {
        return Commands.sequence(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(startPosition.name() + "ToReef" + path.branch())),
                createScoringSequence(),
                AutoUtils.createSource(path.branch(), source));
    }

    private static Command[] createSubsequentPositionCommands(List<PPScoringPath> scoringPaths, String source) {
        Command[] cmds = new Command[scoringPaths.size()];

        for (int i = 1; i < scoringPaths.size(); i++) {
            try {
                cmds[i] = Commands.sequence(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                                "ElevatorStartToReef" + scoringPaths.get(i).branch())),
                        createScoringSequence(),
                        AutoUtils.createSource(scoringPaths.get(i).branch(), source));
            } catch (IOException | ParseException e) {
                throw new AutoLoadException("Error creating auto command", e);
            }
        }

        return cmds;
    }

    private static Command createScoringSequence() {
        return Commands.sequence(
                CommandUtils.finishOnInterrupt(AutoUtils.alignToReefL4().withTimeout(ALIGN_TIMEOUT)),
                new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(SHOOT_TIMEOUT)));
    }

    public static class AutoLoadException extends RuntimeException {
        public AutoLoadException(String message, Throwable cause) {
            super(message, cause);
        }
    }
}
