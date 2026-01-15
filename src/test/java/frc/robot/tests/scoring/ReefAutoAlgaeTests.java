package frc.robot.tests.scoring;

import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static frc.robot.tests.TestControlBridge.Button.*;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.DynamicTest.dynamicTest;
import static utils.Assertions.*;
import static utils.TestRobot.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.Game.*;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlgae;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import java.util.Arrays;
import java.util.stream.Stream;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;

@SuppressWarnings("java:S2187") // dynamic test cases
class ReefAutoAlgaeTests {

    private static final double START_DISTANCE_FROM_REEF = 1.0;

    private static Stream<DynamicTest> testFactory(Stream<AlgaePosition> branches) {
        return branches.map(algaePosition -> {
            // Branch id is the first and last 2 letters of AlgaePosition (BLUE_AB -> BAB)
            String branchId = String.valueOf(algaePosition.name().charAt(0))
                    + String.valueOf(
                            algaePosition.name().charAt(algaePosition.name().length() - 2))
                    + String.valueOf(
                            algaePosition.name().charAt(algaePosition.name().length() - 1));

            return dynamicTest("AutoAlgae " + branchId, () -> {
                testUntil(
                        stopOnCommandEnd(AutoAlgae.class::isInstance),
                        null,
                        robot -> {
                            /* robot and field setup */
                            SimulatedArena.getInstance().placeGamePiecesOnField();
                            final Pose2d startPose = algaePosition
                                    .getPose()
                                    .transformBy(new Transform2d(-START_DISTANCE_FROM_REEF, 0, Rotation2d.kZero));
                            RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(startPose);

                            // Odometry reset has to run during periodic to work correctly
                            runFor(2, () -> RobotContainer.poseSensorFusion.setToPose(startPose));

                            setAllianceStationId(AllianceStationID.Blue1);

                            setEnabled(true);
                            notifyNewData();

                            // wait two periodic cycles for pose reset
                            runAfter(2, () -> controlBridge().pressButton(REEF_ALGAE_SIMPLE, 1));
                        },
                        () -> {
                            assertTrue(
                                    RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.ALGAE),
                                    "Game piece expected to be at least ALGAE, got "
                                            + RobotContainer.elevatorHead
                                                    .getGamePiece()
                                                    .toString());
                            assertReefAlgaeExactlyMissing(branchId);
                        });
            });
        });
    }

    static class Blue {
        @TestFactory
        Stream<DynamicTest> blue() {
            return testFactory(
                    Arrays.stream(AlgaePosition.values()).filter(c -> c.name().startsWith("BLUE")));
        }
    }

    static class Red {
        @TestFactory
        Stream<DynamicTest> red() {
            return testFactory(
                    Arrays.stream(AlgaePosition.values()).filter(c -> c.name().startsWith("RED")));
        }
    }
}
