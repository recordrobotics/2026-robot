package frc.robot.tests.scoring;

import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static frc.robot.tests.TestControlBridge.Button.*;
import static org.junit.jupiter.api.Assertions.*;
import static utils.Assertions.*;
import static utils.TestRobot.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoScore;
import frc.robot.control.AbstractControl.ReefLevelSwitchValue;
import frc.robot.tests.TestControlBridge.Axis;
import frc.robot.utils.ConsoleLogger;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.Test;

class AutoScoreBackawayTests {

    private static final double START_DISTANCE_FROM_REEF = 1.0;

    @Test
    void testBackawaySpinLeft() {

        final Pose2d reefPose = CoralPosition.BLUE_A.getPose();
        final Pose2d startPose = reefPose.transformBy(new Transform2d(-START_DISTANCE_FROM_REEF, 0, Rotation2d.kZero));

        testUntil(
                stopOnCommandEnd(AutoScore.class::isInstance),
                () -> {
                    // Wait for coral to get shot
                    if (SimulatedArena.getInstance().gamePieceLaunched().stream()
                            .anyMatch(s -> s.getType().equals("Coral"))) {
                        controlBridge().setAxis(Axis.X, -1.0);
                        controlBridge().setAxis(Axis.TWIST, 0.7);
                    }
                    return true;
                },
                robot -> {
                    /* robot and field setup */

                    RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(startPose);

                    // Odometry reset has to run during periodic to work correctly
                    runFor(2, () -> RobotContainer.poseSensorFusion.setToPose(startPose));

                    controlBridge().setReefLevel(ReefLevelSwitchValue.L4);

                    setAllianceStationId(AllianceStationID.Blue1);
                    setEnabled(true);
                    notifyNewData();

                    // Give robot preload
                    try {
                        RobotContainer.elevatorHead.getSimIO().setPreload();
                    } catch (Exception e) {
                        ConsoleLogger.logError("Failed to give robot preload", e);
                    }

                    // wait two periodic cycles for pose reset and coral sensor
                    runAfter(2, () -> controlBridge().pressButton(AUTO_SCORE, 1));
                },
                () -> {
                    assertReefCoralExactly("BA4");

                    Pose2d endPose =
                            RobotContainer.drivetrain.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
                    double distanceFromReefPose = endPose.getTranslation().getDistance(reefPose.getTranslation());
                    assertTrue(
                            distanceFromReefPose > 2.0,
                            "Distance from reef after backaway with manual drive to the left should be far");
                });
    }
}
