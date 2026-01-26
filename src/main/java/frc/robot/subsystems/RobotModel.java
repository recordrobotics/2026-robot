package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Represents the physical model of the robot, including mechanisms and their positions */
public final class RobotModel extends ManagedSubsystemBase {

    public interface RobotMechanism {
        int getPoseCount();

        void updatePoses(Pose3d[] poses, int i);
    }

    public static class RobotGamePiece {
        private Supplier<Pose3d> poseSupplier;

        public RobotGamePiece(Supplier<Pose3d> poseSupplier) {
            this.poseSupplier = poseSupplier;
        }

        public Supplier<Pose3d> getPoseSupplier() {
            return poseSupplier;
        }

        public Pose3d getPose() {
            return poseSupplier.get();
        }

        public void setPoseSupplier(Supplier<Pose3d> poseSupplier) {
            this.poseSupplier = poseSupplier;
        }
    }

    public final Intake intake = new Intake();

    @AutoLogLevel(level = Level.REAL)
    public Pose3d[] mechanismPoses = new Pose3d[Intake.POSE_COUNT];

    private final RobotGamePiece robotCoral = new RobotGamePiece(() -> null);
    private final RobotGamePiece robotAlgae = new RobotGamePiece(() -> null);

    public RobotModel() {
        periodicManaged();
    }

    @Override
    public void periodicManaged() {
        updatePoses(intake);

        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.DEBUG_SIM)) {
            Logger.recordOutput("IGamePositions", IGamePosition.aggregatePositions());
            FieldIntersection.logPolygons();
        }
    }

    private void updatePoses(RobotMechanism... mechanisms) {
        int i = 0;
        for (RobotMechanism mechanism : mechanisms) {
            if (i >= mechanismPoses.length) {
                ConsoleLogger.logError("RobotModel.updatePoses: too many mechanisms");
                break;
            }

            mechanism.updatePoses(mechanismPoses, i);
            i += mechanism.getPoseCount();
        }
    }

    @AutoLogLevel(level = Level.SIM)
    public Pose3d[] getFuelPositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3d> fuelPoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Fuel");
            List<Pose3d> robotFuelPoses = Collections.emptyList(); // TODO get from hopper and intake
            if (robotFuelPoses != null) {
                fuelPoses.addAll(robotFuelPoses);
            }
            return fuelPoses.toArray(new Pose3d[0]);
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

    public RobotGamePiece getRobotCoral() {
        return robotCoral;
    }

    public RobotGamePiece getRobotAlgae() {
        return robotAlgae;
    }

    public static class Intake implements RobotMechanism {
        public static final int POSE_COUNT = 1;

        private static final Translation3d SHAFT_ORIGIN = new Translation3d(0, 0.3337, 0.3598);
        private static final double LINE_WIDTH = 3;

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanism =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNode = mechanism.getRoot(
                "intake_root",
                Constants.Intake.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.Intake.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d intakeNode = rootNode.append(new LoggedMechanismLigament2d(
                "intake",
                Constants.Intake.LENGTH,
                Constants.Intake.ANGLE_OFFSET,
                LINE_WIDTH,
                new Color8Bit(Color.kPurple)));

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanismSetpoint =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNodeSetpoint = mechanismSetpoint.getRoot(
                "intake_root",
                Constants.Intake.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.Intake.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d intakeNodeSetpoint = rootNodeSetpoint.append(new LoggedMechanismLigament2d(
                "intake",
                Constants.Intake.LENGTH,
                Constants.Intake.ANGLE_OFFSET,
                LINE_WIDTH,
                new Color8Bit(Color.kViolet)));

        public void update(double angle) {
            intakeNode.setAngle(Units.radiansToDegrees(Constants.Intake.ANGLE_OFFSET + angle));
        }

        public void updateSetpoint(double angle) {
            intakeNodeSetpoint.setAngle(Units.radiansToDegrees(Constants.Intake.ANGLE_OFFSET + angle));
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = Pose3d.kZero.rotateAround(
                    SHAFT_ORIGIN, new Rotation3d(Units.degreesToRadians(intakeNode.getAngle()), 0, 0));
        }
    }
}
