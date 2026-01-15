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
import frc.robot.Constants.Game.AlgaePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlgae;
import frc.robot.tests.TestControlBridge.Axis;
import org.junit.jupiter.api.Test;

class AutoAlgaeBackawayTests {

    private static final double START_DISTANCE_FROM_REEF = 1.0;

    @Test
    void testBackawaySpinLeft() {

        final Pose2d reefPose = AlgaePosition.BLUE_AB.getPose();
        final Pose2d startPose = reefPose.transformBy(new Transform2d(-START_DISTANCE_FROM_REEF, 0, Rotation2d.kZero));

        testUntil(
                stopOnCommandEnd(AutoAlgae.class::isInstance),
                null,
                robot -> {
                    /* robot and field setup */

                    RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(startPose);

                    // Odometry reset has to run during periodic to work correctly
                    runFor(2, () -> RobotContainer.poseSensorFusion.setToPose(startPose));

                    setAllianceStationId(AllianceStationID.Blue1);
                    setEnabled(true);
                    notifyNewData();

                    // wait two periodic cycles for pose reset
                    runAfter(2, () -> {
                        controlBridge().pressButton(REEF_ALGAE_SIMPLE, 2);
                        controlBridge().setAxis(Axis.X, -1.0);
                        controlBridge().setAxis(Axis.TWIST, 0.7);
                    });
                },
                () -> {
                    assertReefAlgaeExactlyMissing("BAB");

                    Pose2d endPose =
                            RobotContainer.drivetrain.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
                    double distanceFromReefPose = endPose.getTranslation().getDistance(reefPose.getTranslation());
                    assertTrue(
                            distanceFromReefPose > 2.0,
                            "Distance from reef after backaway with manual drive to the left should be far");
                });
    }
}
