package frc.robot.subsystems.lights;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public final class StateVisualizerLights extends VirtualLightsSubsystem {

    private static final int VIEW_START = 0;
    private static final int VIEW_END = 29;

    public StateVisualizerLights(Lights lights) {
        super(lights, VIEW_START, VIEW_END);
        setDefaultCommand(
                runPattern(Constants.Lights.ALLIANCE_COLOR_FANCY_WITH_CLIMB).ignoringDisable(true));
    }
}
