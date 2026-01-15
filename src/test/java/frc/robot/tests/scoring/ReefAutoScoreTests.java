package frc.robot.tests.scoring;

import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static frc.robot.tests.TestControlBridge.Button.*;
import static org.junit.jupiter.api.DynamicTest.dynamicTest;
import static utils.Assertions.*;
import static utils.TestRobot.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.Game.*;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoScore;
import frc.robot.control.AbstractControl.ReefLevelSwitchValue;
import frc.robot.utils.ConsoleLogger;
import java.util.Arrays;
import java.util.stream.Stream;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;

@SuppressWarnings("java:S2187") // dynamic test cases
class ReefAutoScoreTests {

    private static final double START_DISTANCE_FROM_REEF = 2.0;

    private static Stream<DynamicTest> testFactory(Stream<CoralPosition> branches, int level) {
        return branches.map(coralPosition -> {
            // Branch id is the first and last letter of CoralPosition + the level (BlueA, 4 -> BA4)
            String branchId = String.valueOf(coralPosition.name().charAt(0))
                    + String.valueOf(
                            coralPosition.name().charAt(coralPosition.name().length() - 1))
                    + level;

            return dynamicTest("AutoScore " + branchId, () -> {
                testUntil(
                        delayStop(level == 1 ? 20 : 0, stopOnCommandEnd(AutoScore.class::isInstance)),
                        null,
                        robot -> {
                            /* robot and field setup */

                            final Pose2d startPose = coralPosition
                                    .getPose()
                                    .transformBy(new Transform2d(-START_DISTANCE_FROM_REEF, 0, Rotation2d.kZero));
                            RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(startPose);

                            // Odometry reset has to run during periodic to work correctly
                            runFor(2, () -> RobotContainer.poseSensorFusion.setToPose(startPose));

                            controlBridge().setReefLevel(ReefLevelSwitchValue.valueOf("L" + level));

                            setAllianceStationId(AllianceStationID.Blue1);
                            setEnabled(true);
                            notifyNewData();

                            if (level != 1) {
                                // Give robot preload
                                try {
                                    RobotContainer.elevatorHead.getSimIO().setPreload();
                                } catch (Exception e) {
                                    ConsoleLogger.logError("Failed to give robot preload", e);
                                }
                            } else {
                                // Give robot preload
                                try {
                                    RobotContainer.coralIntake.getSimIO().addCoral();
                                } catch (Exception e) {
                                    ConsoleLogger.logError("Failed to give robot preload (coral intake)", e);
                                }
                            }

                            // wait two periodic cycles for pose reset and coral sensor
                            runAfter(2, () -> controlBridge().pressButton(AUTO_SCORE, 1));
                        },
                        () -> assertReefCoralExactly(branchId));
            });
        });
    }

    static class Blue1 {
        @TestFactory
        Stream<DynamicTest> blue1() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("BLUE")), 1);
        }
    }

    static class Blue2 {
        @TestFactory
        Stream<DynamicTest> blue2() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("BLUE")), 2);
        }
    }

    static class Blue3 {
        @TestFactory
        Stream<DynamicTest> blue3() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("BLUE")), 3);
        }
    }

    static class Blue4 {
        @TestFactory
        Stream<DynamicTest> blue4() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("BLUE")), 4);
        }
    }

    static class Red1 {
        @TestFactory
        Stream<DynamicTest> red1() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("RED")), 1);
        }
    }

    static class Red2 {
        @TestFactory
        Stream<DynamicTest> red2() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("RED")), 2);
        }
    }

    static class Red3 {
        @TestFactory
        Stream<DynamicTest> red3() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("RED")), 3);
        }
    }

    static class Red4 {
        @TestFactory
        Stream<DynamicTest> red4() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("RED")), 4);
        }
    }
}
