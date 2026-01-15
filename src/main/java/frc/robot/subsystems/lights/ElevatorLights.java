package frc.robot.subsystems.lights;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public final class ElevatorLights extends VirtualLightsSubsystem {

    private static final int VIEW_START = 46;
    private static final int VIEW_END = 93;

    public ElevatorLights(Lights lights) {
        super(lights, VIEW_START, VIEW_END);
        setDefaultCommand(
                runPattern(Constants.Lights.ALLIANCE_COLOR_FANCY_WITH_CLIMB).ignoringDisable(true));
    }
}
