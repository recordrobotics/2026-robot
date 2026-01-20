package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.field.FieldIntersection;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

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

    @AutoLogLevel(level = Level.REAL)
    public Pose3d[] mechanismPoses = new Pose3d[0];

    private final RobotGamePiece robotCoral = new RobotGamePiece(() -> null);
    private final RobotGamePiece robotAlgae = new RobotGamePiece(() -> null);

    public RobotModel() {
        periodicManaged();
    }

    @Override
    public void periodicManaged() {
        updatePoses();

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
    public Pose3d[] getCoralPositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3d> coralPoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Coral");
            Pose3d robotCoralPose = robotCoral.poseSupplier.get();
            if (robotCoralPose != null) {
                coralPoses.add(robotCoralPose);
            }
            return coralPoses.toArray(new Pose3d[0]);
        } else {
            return new Pose3d[0];
        }
    }

    @AutoLogLevel(level = Level.SIM)
    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    public Pose3d[] getAlgaePositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3d> algaePoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Algae");
            Pose3d robotAlgaePose = robotAlgae.poseSupplier.get();
            if (robotAlgaePose != null) {
                algaePoses.add(robotAlgaePose);
            }
            return algaePoses.toArray(new Pose3d[0]);
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
}
