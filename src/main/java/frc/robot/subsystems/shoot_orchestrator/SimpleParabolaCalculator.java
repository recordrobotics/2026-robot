package frc.robot.subsystems.shoot_orchestrator;

import frc.robot.Constants;

public class SimpleParabolaCalculator implements ShotCalculator {
    @Override
    public ForwardReturnType calculateForwardHub(double distanceFromTarget) {
        return calculateForward(distanceFromTarget, Constants.Game.HUB_RIM_HEIGHT_METERS); // TODO should this have `+ Constants.Game.FUEL_DIAMETER_METERS / 2`?
    }

    @Override
    public ForwardReturnType calculateForwardPassing(double distanceFromTarget) {
        return calculateForward(distanceFromTarget, Constants.Game.FUEL_DIAMETER_METERS / 2);
    }

    private ForwardReturnType calculateForward(double distanceFromTarget, double targetHeight) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'calculateForward'");
    }

    @Override
    public double calculateBackwardHub(double hoodAngleRads, double flywheelVelocityMps) {
        return calculateBackward(hoodAngleRads, flywheelVelocityMps, Constants.Game.HUB_RIM_HEIGHT_METERS); // TODO should this have `+ Constants.Game.FUEL_DIAMETER_METERS / 2`?
    }

    @Override
    public double calculateBackwardPassing(double hoodAngleRads, double flywheelVelocityMps) {
        return calculateBackward(hoodAngleRads, flywheelVelocityMps, Constants.Game.FUEL_DIAMETER_METERS / 2);
    }

    private double calculateBackward(double hoodAngleRads, double flywheelVelocityMps, double targetHeight) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'calculateBackward'");
    }
}
