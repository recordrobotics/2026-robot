package frc.robot.subsystems.shoot_orchestrator;

public interface ShotCalculator {
    record ForwardReturnType(
            double hoodAngleRads,
            double flywheelVelocityMps,
            double timeOfFlight,
            double fuelVelocityXMps,
            double fuelVelocityZMps) {}

    public ForwardReturnType calculateForwardHub(double distanceFromTarget); // aims for optimal hub shot, at hub height

    public ForwardReturnType calculateForwardPassing(
            double distanceFromTarget); // aims for optimal passing shot, at floor height

    public double calculateBackwardHub(
            double hoodAngleRads,
            double flywheelVelocityMps); // inverse of calculateForwardHub, used to calculate probability of shot going
    // in

    public double calculateBackwardPassing(
            double hoodAngleRads,
            double flywheelVelocityMps); // inverse of calculateForwardPassing, used to calculate probability of shot
    // hitting target
}
