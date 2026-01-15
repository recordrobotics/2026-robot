package frc.robot.commands.simulation;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;
import java.util.function.Supplier;

public final class PoseAnimator extends Command {

    /** The timer used for waiting. */
    private final Timer timer = new Timer();

    private final double duration;

    private final Pose3d startPose;
    private final Supplier<Pose3d> endPoseSupplier;

    private final Consumer<Pose3d> poseConsumer;

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param seconds the time to wait, in seconds
     */
    @SuppressWarnings("this-escape")
    public PoseAnimator(
            Pose3d startPose, Supplier<Pose3d> endPoseSupplier, Consumer<Pose3d> poseConsumer, double seconds) {
        duration = seconds;
        this.startPose = startPose;
        this.endPoseSupplier = endPoseSupplier;
        this.poseConsumer = poseConsumer;

        SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
    }

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param time the time to wait
     */
    public PoseAnimator(Pose3d startPose, Supplier<Pose3d> endPoseSupplier, Consumer<Pose3d> poseConsumer, Time time) {
        this(startPose, endPoseSupplier, poseConsumer, time.in(Seconds));
    }

    public Pose3d getPose() {
        return startPose.interpolate(endPoseSupplier.get(), timer.get() / duration);
    }

    @Override
    public void initialize() {
        timer.restart();
        poseConsumer.accept(getPose());
    }

    @Override
    public void execute() {
        poseConsumer.accept(getPose());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("duration", () -> duration, null);
    }
}
