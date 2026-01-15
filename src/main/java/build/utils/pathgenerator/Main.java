package build.utils.pathgenerator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.Game.AlgaePosition;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.utils.ConsoleLogger;
import java.util.stream.Stream;

public final class Main {

    public static final Distance FIRST_STAGE_OFFSET = Meters.of(-0.6);

    private Main() {}

    public static void main(String... args) {
        for (CoralPosition coral : Stream.of(CoralPosition.values())
                .filter(c -> c.name().startsWith("Blue"))
                .toList()) {
            try {
                PathHelper.editLinkedWaypoint(
                        "Reef" + coral.name().substring("Blue".length()),
                        coral.getFirstStagePose()
                                .transformBy(new Transform2d(FIRST_STAGE_OFFSET, Meters.of(0), Rotation2d.kZero)));
            } catch (Exception e) {
                ConsoleLogger.logError(e);
            }
        }

        for (AlgaePosition algae : Stream.of(AlgaePosition.values())
                .filter(c -> c.name().startsWith("Blue"))
                .toList()) {
            try {
                PathHelper.editLinkedWaypoint("Reef" + algae.name().substring("Blue".length()), algae.getPose());
            } catch (Exception e) {
                ConsoleLogger.logError(e);
            }
        }

        System.out.println("PathGenerator finished with " + PathHelper.getCounter() + " waypoints updated.");
    }
}
