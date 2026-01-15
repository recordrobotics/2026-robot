package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.RobotGamePiece;
import frc.robot.utils.ConsoleLogger;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class CoralIntakeToElevator extends SequentialCommandGroup implements SimulationCommand {

    private static final double SOURCE_INTAKE_MAX_CORAL_DISTANCE = 0.3;

    private RobotGamePiece coral;

    public CoralIntakeToElevator() {
        if (Constants.RobotState.getMode() == Mode.REAL) return;

        addCommands(
                new InstantCommand(() -> coral = RobotContainer.model.getRobotCoral()),
                Commands.defer(this::createCoralAnimationCommand, Set.of()));
    }

    private Command createCoralAnimationCommand() {
        Supplier<Command> animationSequence = this::createAnimationSequence;
        return coral.getPose() == null
                ? createWaitForCoralCommand().andThen(Commands.defer(animationSequence, Set.of()))
                : animationSequence.get();
    }

    private Command createAnimationSequence() {
        return createToEjectPoseAnimation()
                .andThen(createToEjectFinalPoseAnimation())
                .andThen(createToChannelPoseAnimation())
                .andThen(createToShooterTargetAnimation())
                .andThen(this::setFinalCoralState);
    }

    @SuppressWarnings("java:S109") // specific animation timings
    private Command createToEjectPoseAnimation() {
        return new PoseAnimator(
                coral.getPose().relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                () -> RobotContainer.model
                        .elevator
                        .getCoralIntakeEjectPose()
                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                this::updateCoralPose,
                0.1);
    }

    @SuppressWarnings("java:S109") // specific animation timings
    private Command createToEjectFinalPoseAnimation() {
        return new PoseAnimator(
                RobotContainer.model
                        .elevator
                        .getCoralIntakeEjectPose()
                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                () -> RobotContainer.model
                        .elevator
                        .getCoralIntakeEjectFinalPose()
                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                this::updateCoralPose,
                0.1);
    }

    @SuppressWarnings("java:S109") // specific animation timings
    private Command createToChannelPoseAnimation() {
        return new PoseAnimator(
                RobotContainer.model
                        .elevator
                        .getCoralIntakeEjectFinalPose()
                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                () -> RobotContainer.model
                        .elevator
                        .getCoralIntakeChannelPose()
                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                this::updateCoralPose,
                0.15);
    }

    @SuppressWarnings("java:S109") // specific animation timings
    private Command createToShooterTargetAnimation() {
        return new PoseAnimator(
                RobotContainer.model
                        .elevator
                        .getCoralIntakeChannelPose()
                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                () -> RobotContainer.model
                        .elevatorArm
                        .getCoralShooterTargetPose()
                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                this::updateCoralPose,
                0.4);
    }

    private void updateCoralPose(Pose3d pose) {
        coral.setPoseSupplier(() -> new Pose3d(RobotContainer.model.getRobot())
                .plus(new Transform3d(pose.getTranslation(), pose.getRotation())));
    }

    private void setFinalCoralState() {
        coral.setPoseSupplier(RobotContainer.model.elevatorArm::getCoralShooterTargetPose);
        try {
            RobotContainer.elevatorHead.getSimIO().setCoralDetectorSim(false);
        } catch (Exception e) {
            ConsoleLogger.logError(e);
        }
    }

    private WaitUntilCommand createWaitForCoralCommand() {
        return new WaitUntilCommand(() -> {
            if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
                return false;
            }
            return checkForCoralInSimulation();
        });
    }

    private boolean checkForCoralInSimulation() {
        for (GamePieceProjectile c : SimulatedArena.getInstance().gamePieceLaunched()) {
            if (c.getType().equals("Coral") && isCoralNearTarget(c)) {
                coral.setPoseSupplier(c::getPose3d);
                SimulatedArena.getInstance().removeProjectile(c);
                return true;
            }
        }
        return false;
    }

    private static boolean isCoralNearTarget(GamePieceProjectile coral) {
        Pose3d pose = coral.getPose3d();
        Pose3d targetPose = RobotContainer.model.elevator.getCoralIntakeEjectPose();
        return pose.getTranslation().getDistance(targetPose.getTranslation()) < SOURCE_INTAKE_MAX_CORAL_DISTANCE;
    }

    @Override
    public Command simulateFor(Command command) {
        if (Constants.RobotState.getMode() == Mode.REAL) {
            return command;
        } else {
            return command.alongWith(this);
        }
    }
}
