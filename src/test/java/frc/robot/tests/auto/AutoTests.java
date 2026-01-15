package frc.robot.tests.auto;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.getEnabled;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.getMatchTime;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAutonomous;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setMatchTime;
import static utils.Assertions.*;
import static utils.TestRobot.*;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.BargeLeftAuto;
import frc.robot.commands.auto.BargeRightAuto;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.auto.PlannedAuto.AutoSupplier;
import frc.robot.utils.ConsoleLogger;
import org.junit.jupiter.api.Test;

class AutoTests {

    boolean periodic() {
        if (getEnabled()) {
            setMatchTime(getMatchTime() - 0.02);
            notifyNewData();
        }
        return true;
    }

    private void testAuto(
            AutoSupplier autoSupplier,
            AllianceStationID allianceStation,
            Constants.FieldStartingLocation startLocation,
            String expectedReefSequence) {
        testUntil(
                () -> getMatchTime() <= 0,
                this::periodic,
                robot -> {
                    /* robot and field setup */

                    setAllianceStationId(allianceStation);
                    setAutonomous(true);
                    setMatchTime(15);
                    notifyNewData();

                    PlannedAuto.setAutoSupplier(autoSupplier);

                    final Pose2d startPose = startLocation.getPose();
                    RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(startPose);

                    // Odometry reset has to run during periodic to work correctly
                    runFor(2, () -> RobotContainer.poseSensorFusion.setToPose(startPose));

                    // wait two periodic cycles before enable for odometry reset
                    runAfter(2, () -> {
                        // Give robot preload
                        try {
                            RobotContainer.elevatorHead.getSimIO().setPreload();
                        } catch (Exception e) {
                            ConsoleLogger.logError("Failed to give robot preload", e);
                        }
                        setEnabled(true);
                        notifyNewData();
                    });
                },
                () -> assertReefCoralExactly(expectedReefSequence),
                Seconds.of(16));
    }

    @Test
    void testBargeLeftAutoBlue() {
        testAuto(BargeLeftAuto::new, AllianceStationID.Blue1, Constants.FieldStartingLocation.BARGE_LEFT, "BJ4,BK4");
    }

    @Test
    void testBargeLeftAutoRed() {
        testAuto(BargeLeftAuto::new, AllianceStationID.Red1, Constants.FieldStartingLocation.BARGE_LEFT, "RJ4,RK4");
    }

    @Test
    void testBargeRightAutoBlue() {
        testAuto(BargeRightAuto::new, AllianceStationID.Blue3, Constants.FieldStartingLocation.BARGE_RIGHT, "BE4,BD4");
    }

    @Test
    void testBargeRightAutoRed() {
        testAuto(BargeRightAuto::new, AllianceStationID.Red3, Constants.FieldStartingLocation.BARGE_RIGHT, "RE4,RD4");
    }

    @Test
    void testCenterAutoBlue() {
        testAuto(
                () -> new PathPlannerAuto("CenterWithProcessor"),
                AllianceStationID.Blue2,
                Constants.FieldStartingLocation.BARGE_CENTER,
                "BH4");
    }

    @Test
    void testCenterAutoRed() {
        testAuto(
                () -> new PathPlannerAuto("CenterWithProcessor"),
                AllianceStationID.Red2,
                Constants.FieldStartingLocation.BARGE_CENTER,
                "RH4");
    }

    @Test
    void testCenterAutoButBetterBlue() {
        testAuto(
                () -> new PathPlannerAuto("CenterWithProcessorButBetter"),
                AllianceStationID.Blue2,
                Constants.FieldStartingLocation.BARGE_CENTER,
                "BH4");
    }

    @Test
    void testCenterAutoButBetterRed() {
        testAuto(
                () -> new PathPlannerAuto("CenterWithProcessorButBetter"),
                AllianceStationID.Red2,
                Constants.FieldStartingLocation.BARGE_CENTER,
                "RH4");
    }
}
