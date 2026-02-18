package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.field.FieldIntersection;
import java.util.Collections;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/** Represents the physical model of the robot, including mechanisms and their positions */
public final class RobotModel extends ManagedSubsystemBase {

    public interface MechanismModel {
        int getPoseCount();

        void updatePoses(Pose3d[] poses, int i);
    }

    public final IntakeModel intakeModel = new IntakeModel();
    public final ShooterModel shooterModel = new ShooterModel();
    public final ClimberModel climberModel = new ClimberModel();

    @AutoLogLevel(level = Level.REAL)
    public Pose3d[] mechanismPoses =
            new Pose3d[intakeModel.getPoseCount() + shooterModel.getPoseCount() + climberModel.getPoseCount()];

    public RobotModel() {
        periodicManaged();
    }

    @Override
    public void periodicManaged() {
        updatePoses(intakeModel, shooterModel, climberModel);

        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.DEBUG_SIM)) {
            Logger.recordOutput("IGamePositions", IGamePosition.aggregatePositions());
            FieldIntersection.logAllInstances();
        }
    }

    private void updatePoses(MechanismModel... mechanismModels) {
        int i = 0;
        for (MechanismModel mechanismModel : mechanismModels) {
            if (i >= mechanismPoses.length) {
                ConsoleLogger.logError("RobotModel.updatePoses: too many mechanisms");
                break;
            }

            mechanismModel.updatePoses(mechanismPoses, i);
            i += mechanismModel.getPoseCount();
        }
    }

    @AutoLogLevel(level = Level.SIM)
    public Pose3d[] getFuelPositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3d> arenaFuelPoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Fuel");
            List<Pose3d> robotFuelPoses = Collections.emptyList(); // TODO get from hopper and intake
            arenaFuelPoses.addAll(robotFuelPoses);
            return arenaFuelPoses.toArray(new Pose3d[0]);
        } else {
            return new Pose3d[0];
        }
    }

    @AutoLogLevel(level = Level.SIM)
    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    public Pose2d getRobot() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            return RobotContainer.drivetrain.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
        } else {
            return Pose2d.kZero;
        }
    }

    public static class IntakeModel implements MechanismModel {
        public static final int POSE_COUNT = 2;
        private static final Translation3d SHAFT_ORIGIN = new Translation3d(-0.285662, 0, 0.274755);
        private static final ArmGeometryPoint[] ARM_GEOMETRY = new ArmGeometryPoint[] {
            new ArmGeometryPoint(Units.degreesToRadians(53.532698), Units.degreesToRadians(-51.8490194205), 0.314003),
            new ArmGeometryPoint(Units.degreesToRadians(17.329479), Units.degreesToRadians(-19.7844139869), 0.356156),
            new ArmGeometryPoint(Units.degreesToRadians(0), Units.degreesToRadians(-1.19101124384), 0.352317)
        };
        private static final double HOPPER_TO_AXLE_DISTANCE_METERS = 0.054722;

        private double angleRadians;
        private double hopperExtensionMeters;

        public void update(double newAngleRadians) {
            angleRadians = newAngleRadians;
            hopperExtensionMeters = Math.max(
                    hopperExtensionMeters, ArmGeometryPoint.calculateHopperExtension(angleRadians, ARM_GEOMETRY));
        }

        public void resetHopperExtension() {
            hopperExtensionMeters = 0;
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = Pose3d.kZero.rotateAround(SHAFT_ORIGIN, new Rotation3d(0, angleRadians, 0));
            poses[i + 1] = new Pose3d(-hopperExtensionMeters, 0, 0, Rotation3d.kZero);
        }

        private record ArmGeometryPoint(double lowestArmAngleRadians, double angleOffset, double length) {
            public static ArmGeometryPoint findCurrentPoint(double armAngle, ArmGeometryPoint[] geometry) {
                for (ArmGeometryPoint point : geometry) {
                    if (armAngle >= point.lowestArmAngleRadians) {
                        return point;
                    }
                }
                return geometry[
                        geometry.length - 1]; // if we are below the lowest point, just use the lowest point's geometry
            }

            public static double calculateHopperExtension(double armAngle, ArmGeometryPoint[] geometry) {
                ArmGeometryPoint currentPoint = findCurrentPoint(armAngle, geometry);
                return Math.max(
                        0,
                        currentPoint.length * Math.cos(armAngle + currentPoint.angleOffset)
                                - HOPPER_TO_AXLE_DISTANCE_METERS);
            }
        }
    }

    public static class ShooterModel implements MechanismModel {
        public static final int POSE_COUNT = 2;
        private static final Translation3d TURRET_ORIGIN = new Translation3d(0.12715, 0.12715, 0);
        private static final Translation3d HOOD_LOCAL_ORIGIN = new Translation3d(0.2105, 0, 0.4556);

        private double turretAngleRadians;
        private double hoodAngleRadians;

        public void update(double newTurretAngleRadians, double newHoodAngleRadians) {
            turretAngleRadians = newTurretAngleRadians;
            hoodAngleRadians = newHoodAngleRadians;
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = Pose3d.kZero.rotateAround(TURRET_ORIGIN, new Rotation3d(0, 0, turretAngleRadians));
            poses[i + 1] = poses[i].transformBy(new Transform3d(
                    Translation3d.kZero.rotateAround(HOOD_LOCAL_ORIGIN, new Rotation3d(0, -hoodAngleRadians, 0)),
                    new Rotation3d(0, -hoodAngleRadians, 0)));
        }
    }

    public static class ClimberModel implements MechanismModel {
        public static final int POSE_COUNT = 1;

        private double climberHeightMeters;

        public void update(double newClimberHeightMeters) {
            climberHeightMeters = newClimberHeightMeters;
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = new Pose3d(0, 0, climberHeightMeters, Rotation3d.kZero);
        }
    }
}
