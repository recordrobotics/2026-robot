package frc.robot.tests.manual;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static org.junit.jupiter.api.Assumptions.assumeTrue;
import static utils.TestRobot.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.RobotContainer;
import org.junit.jupiter.api.Test;

class CustomMapleSimPoseTests {

    @Test
    void manualNTRobotPose() {
        assumeTrue(
                Constants.RobotState.UNIT_TESTS_ENABLE_ADVANTAGE_SCOPE,
                "Manual NT Pose only available with networking enabled.");

        double[] initialRotation = new double[1];

        testUntil(
                () -> false,
                () -> {
                    final double x = SmartDashboard.getNumber("CustomPose/X", 0);
                    final double y = SmartDashboard.getNumber("CustomPose/Y", 0);
                    final double rotation = SmartDashboard.getNumber("CustomPose/Rotation", 0);

                    final Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
                    RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(pose);
                    // RobotContainer.drivetrain
                    //         .getSwerveDriveSimulation()
                    //         .getGyroSimulation()
                    //         .setRotation(Rotation2d.fromRadians(pose.getRotation().getRadians() -
                    // initialRotation[0]));
                    RobotContainer.poseSensorFusion.setToPose(
                            pose); // TODO: figure out why kalman filter dies if moving robot around using
                    // setSimulationWorldPose

                    return true;
                },
                robot -> {
                    setAllianceStationId(AllianceStationID.Blue1);
                    setEnabled(true);
                    notifyNewData();

                    final Pose2d startPose = FieldStartingLocation.BARGE_CENTER.getPose();
                    RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(startPose);

                    initialRotation[0] = startPose.getRotation().getRadians();

                    SmartDashboard.putNumber("CustomPose/X", startPose.getX());
                    SmartDashboard.putNumber("CustomPose/Y", startPose.getY());
                    SmartDashboard.putNumber(
                            "CustomPose/Rotation", startPose.getRotation().getDegrees());
                },
                () -> {},
                Seconds.zero());
    }
}
