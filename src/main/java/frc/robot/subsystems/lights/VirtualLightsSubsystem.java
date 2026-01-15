package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lights;
import java.util.function.Supplier;

/** A virtual lights subsystem that represents a subset of the physical lights on the robot. */
public abstract class VirtualLightsSubsystem extends SubsystemBase {

    private final int viewStart;
    private final int viewEnd;

    private final AddressableLEDBufferView view;

    /**
     * Creates a new VirtualLightsSubsystem.
     *
     * @param lights The lights subsystem to get the buffer from.
     * @param viewStart The start index of the view.
     * @param viewEnd The end index of the view.
     */
    protected VirtualLightsSubsystem(Lights lights, int viewStart, int viewEnd) {
        this.viewStart = viewStart;
        this.viewEnd = viewEnd;

        view = lights.getBuffer().createView(viewStart, viewEnd);
    }

    /**
     * Gets the view of the lights.
     *
     * @return The view of the lights.
     */
    public AddressableLEDBufferView getView() {
        return view;
    }

    /**
     * Gets the start index of the view.
     *
     * @return The start index of the view.
     */
    public int getViewStart() {
        return viewStart;
    }

    /**
     * Gets the end index of the view.
     *
     * @return The end index of the view.
     */
    public int getViewEnd() {
        return viewEnd;
    }

    /**
     * Runs a pattern on the view.
     *
     * @param pattern The pattern to run.
     * @return The command that runs the pattern.
     */
    public Command runPattern(LEDPattern pattern) {
        // return run(() -> pattern.atBrightness(Constants.Lights.MULTIPLIER).applyTo(view));
        return new InstantCommand(() -> {}, this);
    }

    /**
     * Runs a pattern on the view.
     *
     * @param patternSupplier The supplier of the pattern to run.
     * @return The command that runs the pattern.
     */
    public Command runPattern(Supplier<LEDPattern> patternSupplier) {
        // return run(() ->
        // patternSupplier.get().atBrightness(Constants.Lights.MULTIPLIER).applyTo(view));
        return new InstantCommand(() -> {}, this);
    }
}
