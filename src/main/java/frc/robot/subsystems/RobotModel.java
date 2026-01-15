package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

    public static class Elevator implements RobotMechanism {
        public static final int POSE_COUNT = 2; /* 2 stage elevator */

        private static final double FIRST_STAGE_MAX_HEIGHT = 0.760967; // meters
        private static final double SECOND_STAGE_MAX_HEIGHT = 1.44; // meters
        private static final double LINE_WIDTH = 10;

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanism =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNode = mechanism.getRoot(
                "elevator_root",
                Constants.Elevator.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.Elevator.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d elevatorNode = rootNode.append(new LoggedMechanismLigament2d(
                "elevator", Constants.Elevator.MIN_LENGTH, 90, LINE_WIDTH, new Color8Bit(Color.kBlue)));

        @SuppressWarnings("unused")
        private LoggedMechanismLigament2d coralShooter = elevatorNode.append(new LoggedMechanismLigament2d(
                "coralShooter",
                Constants.ElevatorHead.HOW_FAR_FORWARDS_FROM_THE_ELEVATOR_IS_THE_CORAL_SHOOTER,
                -90,
                LINE_WIDTH,
                new Color8Bit(Color.kGreen)));

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanismSetpoint =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNodeSetpoint = mechanismSetpoint.getRoot(
                "elevator_root",
                Constants.Elevator.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.Elevator.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d elevatorNodeSetpoint = rootNodeSetpoint.append(new LoggedMechanismLigament2d(
                "elevator", Constants.Elevator.MIN_LENGTH, 90, LINE_WIDTH, new Color8Bit(Color.kBlueViolet)));

        @SuppressWarnings("unused")
        private LoggedMechanismLigament2d coralShooterSetpoint =
                elevatorNodeSetpoint.append(new LoggedMechanismLigament2d(
                        "coralShooter",
                        Constants.ElevatorHead.HOW_FAR_FORWARDS_FROM_THE_ELEVATOR_IS_THE_CORAL_SHOOTER,
                        -90,
                        LINE_WIDTH,
                        new Color8Bit(Color.kGreenYellow)));

        public void update(double height) {
            elevatorNode.setLength(Constants.Elevator.MIN_LENGTH + height);
        }

        public void updateSetpoint(double height) {
            elevatorNodeSetpoint.setLength(Constants.Elevator.MIN_LENGTH + height);
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        public double getFirstStageHeight() {
            return (elevatorNode.getLength() - Constants.Elevator.MIN_LENGTH)
                    / Constants.Elevator.MAX_HEIGHT
                    * FIRST_STAGE_MAX_HEIGHT;
        }

        public double getSecondStageHeight() {
            return (elevatorNode.getLength() - Constants.Elevator.MIN_LENGTH)
                    / Constants.Elevator.MAX_HEIGHT
                    * SECOND_STAGE_MAX_HEIGHT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            // First stage
            poses[i++] = new Pose3d(new Translation3d(0, 0, getFirstStageHeight()), Rotation3d.kZero);

            // Second stage
            poses[i] = new Pose3d(new Translation3d(0, 0, getSecondStageHeight()), Rotation3d.kZero);
        }

        @SuppressWarnings("java:S109") // specific xyz transform coordinates
        public Pose3d getCoralIntakeEjectPose() {
            Pose3d robotOrigin = Pose3d.kZero;
            if (RobotContainer.model != null) robotOrigin = new Pose3d(RobotContainer.model.getRobot());
            return robotOrigin.transformBy(
                    new Transform3d(-0.1, 0.27, 0.494817 + Constants.CoralIntake.LENGTH, Rotation3d.kZero));
        }

        @SuppressWarnings("java:S109") // specific xyz transform coordinates
        public Pose3d getCoralIntakeEjectFinalPose() {
            Pose3d robotOrigin = Pose3d.kZero;
            if (RobotContainer.model != null) robotOrigin = new Pose3d(RobotContainer.model.getRobot());
            return robotOrigin.transformBy(
                    new Transform3d(-0.1, 0.18, 0.456817 + Constants.CoralIntake.LENGTH, Rotation3d.kZero));
        }

        @SuppressWarnings("java:S109") // specific xyz transform coordinates
        public Pose3d getCoralIntakeChannelPose() {
            Pose3d robotOrigin = Pose3d.kZero;
            if (RobotContainer.model != null) robotOrigin = new Pose3d(RobotContainer.model.getRobot());
            return robotOrigin.transformBy(
                    new Transform3d(-0.1, 0.18, 0.316817 + Constants.CoralIntake.LENGTH, Rotation3d.kZero));
        }
    }

    public static class CoralIntake implements RobotMechanism {
        public static final int POSE_COUNT = 1;

        private static final Translation3d SHAFT_ORIGIN = new Translation3d(0, 0.3337, 0.3598);
        private static final double LINE_WIDTH = 3;

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanism =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNode = mechanism.getRoot(
                "coralintake_root",
                Constants.CoralIntake.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.CoralIntake.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d coralIntakeNode = rootNode.append(new LoggedMechanismLigament2d(
                "coralintake",
                Constants.CoralIntake.LENGTH,
                Constants.CoralIntake.ANGLE_OFFSET,
                LINE_WIDTH,
                new Color8Bit(Color.kPurple)));

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanismSetpoint =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNodeSetpoint = mechanismSetpoint.getRoot(
                "coralintake_root",
                Constants.CoralIntake.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.CoralIntake.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d coralIntakeNodeSetpoint =
                rootNodeSetpoint.append(new LoggedMechanismLigament2d(
                        "coralintake",
                        Constants.CoralIntake.LENGTH,
                        Constants.CoralIntake.ANGLE_OFFSET,
                        LINE_WIDTH,
                        new Color8Bit(Color.kViolet)));

        public void update(double angle) {
            coralIntakeNode.setAngle(Units.radiansToDegrees(Constants.CoralIntake.ANGLE_OFFSET + angle));
        }

        public void updateSetpoint(double angle) {
            coralIntakeNodeSetpoint.setAngle(Units.radiansToDegrees(Constants.CoralIntake.ANGLE_OFFSET + angle));
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = Pose3d.kZero.rotateAround(
                    SHAFT_ORIGIN, new Rotation3d(Units.degreesToRadians(coralIntakeNode.getAngle()), 0, 0));
        }

        @SuppressWarnings("java:S109") // specific xyz transform coordinates
        public Pose3d getCoralTargetPose() {
            Pose3d robotOrigin = Pose3d.kZero;
            if (RobotContainer.model != null) robotOrigin = new Pose3d(RobotContainer.model.getRobot());
            Pose3d coralIntakePose = robotOrigin.transformBy(new Transform3d(
                    SHAFT_ORIGIN, new Rotation3d(Units.degreesToRadians(coralIntakeNode.getAngle()), 0, 0)));

            return coralIntakePose.transformBy(
                    new Transform3d(-0.1, Constants.CoralIntake.LENGTH, 0.038, Rotation3d.kZero));
        }
    }

    public static class ElevatorArm implements RobotMechanism {
        public static final int POSE_COUNT = 1;

        private static final Translation3d SHAFT_ORIGIN = new Translation3d(0.318, 0, 0.575);
        private static final double CORAL_ANGLE_IN_SHOOTER = 20;
        private static final double LINE_WIDTH = 3;

        private RobotModel model;

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanism =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNode = mechanism.getRoot(
                "elevatorarm_root",
                Constants.ElevatorArm.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.ElevatorArm.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d elevatorArmNode = rootNode.append(new LoggedMechanismLigament2d(
                "elevatorarm",
                Constants.ElevatorArm.LENGTH,
                Constants.ElevatorArm.ANGLE_OFFSET,
                LINE_WIDTH,
                new Color8Bit(Color.kPurple)));

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanismSetpoint =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNodeSetpoint = mechanismSetpoint.getRoot(
                "elevatorarm_root",
                Constants.ElevatorArm.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.ElevatorArm.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d elevatorArmNodeSetpoint =
                rootNodeSetpoint.append(new LoggedMechanismLigament2d(
                        "elevatorarm",
                        Constants.ElevatorArm.LENGTH,
                        Constants.ElevatorArm.ANGLE_OFFSET,
                        LINE_WIDTH,
                        new Color8Bit(Color.kViolet)));

        public ElevatorArm(RobotModel model) {
            this.model = model;
        }

        public void update(double angle) {
            elevatorArmNode.setAngle(Units.radiansToDegrees(Constants.ElevatorArm.ANGLE_OFFSET + angle));
        }

        public void updateSetpoint(double angle) {
            elevatorArmNodeSetpoint.setAngle(Units.radiansToDegrees(Constants.ElevatorArm.ANGLE_OFFSET + angle));
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            Pose3d pose = Pose3d.kZero.rotateAround(
                    SHAFT_ORIGIN, new Rotation3d(0, -Units.degreesToRadians(elevatorArmNode.getAngle()), 0));
            poses[i] = new Pose3d(
                    pose.getTranslation().plus(new Translation3d(0, 0, model.elevator.getSecondStageHeight())),
                    pose.getRotation());
        }

        @SuppressWarnings("java:S109") // specific xyz transform coordinates
        public Pose3d getCoralShooterTargetPose() {
            Pose3d robotOrigin = Pose3d.kZero;
            if (RobotContainer.model != null) robotOrigin = new Pose3d(RobotContainer.model.getRobot());

            Pose3d pose = new Pose3d(SHAFT_ORIGIN.plus(new Translation3d(0.166, 0.19, -0.04)), Rotation3d.kZero)
                    .rotateAround(
                            SHAFT_ORIGIN, new Rotation3d(0, -Units.degreesToRadians(elevatorArmNode.getAngle()), 0));
            pose = new Pose3d(
                    pose.getTranslation().plus(new Translation3d(0, 0, model.elevator.getSecondStageHeight())),
                    pose.getRotation());

            Pose3d coralShooterPose =
                    robotOrigin.transformBy(new Transform3d(pose.getTranslation(), pose.getRotation()));

            return coralShooterPose.transformBy(new Transform3d(
                    0, 0, 0, new Rotation3d(0, Units.degreesToRadians(180 + 90 + CORAL_ANGLE_IN_SHOOTER), 0)));
        }

        @SuppressWarnings("java:S109") // specific xyz transform coordinates
        public Pose3d getAlgaeGrabberTargetPoseTop() {
            Pose3d robotOrigin = new Pose3d();
            if (RobotContainer.model != null) robotOrigin = new Pose3d(RobotContainer.model.getRobot());

            Pose3d pose = new Pose3d(SHAFT_ORIGIN.plus(new Translation3d(0.18, -0.06, 0.1)), new Rotation3d())
                    .rotateAround(
                            SHAFT_ORIGIN, new Rotation3d(0, -Units.degreesToRadians(elevatorArmNode.getAngle()), 0));
            pose = new Pose3d(
                    pose.getTranslation()
                            .plus(new Translation3d(
                                    0, 0, model.elevator.elevatorNode.getLength() - Constants.Elevator.MIN_LENGTH)),
                    pose.getRotation());

            Pose3d algaeGrabberPose =
                    robotOrigin.transformBy(new Transform3d(pose.getTranslation(), pose.getRotation()));

            return algaeGrabberPose.transformBy(new Transform3d(0, 0, 0, new Rotation3d()));
        }

        @SuppressWarnings("java:S109") // specific xyz transform coordinates
        public Pose3d getAlgaeGrabberTargetPoseBottom() {
            Pose3d robotOrigin = new Pose3d();
            if (RobotContainer.model != null) robotOrigin = new Pose3d(RobotContainer.model.getRobot());

            Pose3d pose = new Pose3d(SHAFT_ORIGIN.plus(new Translation3d(0.18, -0.11, -0.1)), new Rotation3d())
                    .rotateAround(
                            SHAFT_ORIGIN, new Rotation3d(0, -Units.degreesToRadians(elevatorArmNode.getAngle()), 0));
            pose = new Pose3d(
                    pose.getTranslation()
                            .plus(new Translation3d(
                                    0, 0, model.elevator.elevatorNode.getLength() - Constants.Elevator.MIN_LENGTH)),
                    pose.getRotation());

            Pose3d algaeGrabberPose =
                    robotOrigin.transformBy(new Transform3d(pose.getTranslation(), pose.getRotation()));

            return algaeGrabberPose.transformBy(new Transform3d(0, 0, 0, new Rotation3d()));
        }
    }

    public static class Climber implements RobotMechanism {
        public static final int POSE_COUNT = 1;

        private static final Translation3d SHAFT_ORIGIN = new Translation3d(-0.2921, 0, 0.4097);
        private static final double LINE_WIDTH = 3;

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanism =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNode = mechanism.getRoot(
                "climber_root",
                Constants.Climber.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.Climber.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d climberNode = rootNode.append(new LoggedMechanismLigament2d(
                "climber",
                Constants.Climber.LENGTH.in(Meters),
                Constants.Climber.ANGLE_OFFSET,
                LINE_WIDTH,
                new Color8Bit(Color.kPurple)));

        @AutoLogLevel(level = Level.DEBUG_REAL)
        private LoggedMechanism2d mechanismSetpoint =
                new LoggedMechanism2d(Constants.Frame.FRAME_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

        private LoggedMechanismRoot2d rootNodeSetpoint = mechanismSetpoint.getRoot(
                "climber_root",
                Constants.Climber.ROOT_MECHANISM_POSE.getX() + Constants.Frame.FRAME_WIDTH / 2.0,
                Constants.Climber.ROOT_MECHANISM_POSE.getY());
        private LoggedMechanismLigament2d climberNodeSetpoint = rootNodeSetpoint.append(new LoggedMechanismLigament2d(
                "climber",
                Constants.Climber.LENGTH.in(Meters),
                Constants.Climber.ANGLE_OFFSET,
                LINE_WIDTH,
                new Color8Bit(Color.kViolet)));

        public void update(double angle) {
            climberNode.setAngle(Units.radiansToDegrees(Constants.Climber.ANGLE_OFFSET + angle));
        }

        public void updateSetpoint(double angle) {
            climberNodeSetpoint.setAngle(Units.radiansToDegrees(Constants.Climber.ANGLE_OFFSET + angle));
        }

        @Override
        public int getPoseCount() {
            return POSE_COUNT;
        }

        @Override
        public void updatePoses(Pose3d[] poses, int i) {
            poses[i] = new Pose3d(0, 0, 0, new Rotation3d())
                    .rotateAround(SHAFT_ORIGIN, new Rotation3d(0, Units.degreesToRadians(climberNode.getAngle()), 0));
        }
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

    public final Elevator elevator = new Elevator();
    public final ElevatorArm elevatorArm = new ElevatorArm(this);
    public final Climber climber = new Climber();
    public final CoralIntake coralIntake = new CoralIntake();

    private final RobotGamePiece robotCoral = new RobotGamePiece(() -> null);
    private final RobotGamePiece robotAlgae = new RobotGamePiece(() -> null);

    @AutoLogLevel(level = Level.REAL)
    public Pose3d[] mechanismPoses =
            new Pose3d[Elevator.POSE_COUNT + ElevatorArm.POSE_COUNT + CoralIntake.POSE_COUNT + Climber.POSE_COUNT];

    public RobotModel() {
        periodicManaged();
    }

    @Override
    public void periodicManaged() {
        updatePoses(elevator, elevatorArm, coralIntake, climber);

        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.DEBUG_SIM)) {
            Logger.recordOutput(
                    "IGamePositions",
                    IGamePosition.aggregatePositions(
                            Constants.Game.CoralPosition.values(),
                            Constants.Game.AlgaePosition.values(),
                            Constants.Game.SourcePosition.values(),
                            Constants.Game.ProcessorPosition.values()));
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
