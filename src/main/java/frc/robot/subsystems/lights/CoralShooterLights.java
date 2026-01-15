package frc.robot.subsystems.lights;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public final class CoralShooterLights extends VirtualLightsSubsystem {

    private static final int VIEW_START = 112;
    private static final int VIEW_END = 117;

    public CoralShooterLights(Lights lights) {
        super(lights, VIEW_START, VIEW_END);
        setDefaultCommand(
                runPattern(Constants.Lights.ALLIANCE_COLOR_FANCY_WITH_CLIMB).ignoringDisable(true));
    }
}
