package frc.robot.subsystems.shootorchestrator;

public interface ShotCalculator {
    record ForwardReturnType(
            double hoodAngleRads,
            double flywheelVelocityMps,
            double timeOfFlight,
            double fuelVelocityXMps,
            double fuelVelocityZMps) {}

    ForwardReturnType calculateForwardHub(double distanceFromTarget); // aims for optimal hub shot, at hub height

    ForwardReturnType calculateForwardPassing(
            double distanceFromTarget); // aims for optimal passing shot, at floor height

    double calculateBackwardHub(
            double hoodAngleRads,
            double flywheelVelocityMps); // inverse of calculateForwardHub, used to calculate probability of shot going
    // in

    double calculateBackwardPassing(
            double hoodAngleRads,
            double flywheelVelocityMps); // inverse of calculateForwardPassing, used to calculate probability of shot
    // hitting target
}
