package frc.robot.subsystems.shootorchestrator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PassingRegressionCalculator implements ShotCalculator {

    private static final double FLYWHEEL_TO_FUEL_RATIO = 0.38981;

    @Override
    public double fuelToFlywheelVelocity(double fuelVelocityMps) {
        double ratio = SmartDashboard.getNumber("FLYWHEEL_RATIO", FLYWHEEL_TO_FUEL_RATIO);
        return fuelVelocityMps / ratio;
    }

    @Override
    public double flywheelToFuelVelocity(double flywheelVelocityMps) {
        double ratio = SmartDashboard.getNumber("FLYWHEEL_RATIO", FLYWHEEL_TO_FUEL_RATIO);
        return flywheelVelocityMps * ratio;
    }

    @Override
    public ShotCalculation calculateShot(double distanceToTargetMeters, double robotVelocityTowardsTargetMps) {
        return new ShotCalculation(
                Math.toRadians(calculateAngle(distanceToTargetMeters, robotVelocityTowardsTargetMps)),
                calculateBallVelocity(distanceToTargetMeters, robotVelocityTowardsTargetMps),
                calculateTimeOfFlight(distanceToTargetMeters, robotVelocityTowardsTargetMps),
                calculateAllowableShootAngleError(distanceToTargetMeters, robotVelocityTowardsTargetMps),
                calculateAllowableVelocityMagnitudeError(distanceToTargetMeters, robotVelocityTowardsTargetMps));
    }

    public static double calculateAngle(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 49.859734928
                + -0.272845617732 * robotVelocity
                + -0.991148634043 * robotVelocity2
                + -0.0826117216629 * robotVelocity3
                + 0.00638104079757 * robotVelocity4
                + 2.86813932138 * distance
                + 1.30028666575 * distance * robotVelocity
                + 0.195448844706 * distance * robotVelocity2
                + 0.00557291128529 * distance * robotVelocity3
                + -0.689082931181 * distance2
                + -0.157559202891 * distance2 * robotVelocity
                + -0.009739821626 * distance2 * robotVelocity2
                + 0.0482569691739 * distance3
                + 0.00515810131838 * distance3 * robotVelocity
                + -0.00104220022003 * distance4;
    }

    public static double calculateBallVelocity(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 2.22626774077
                + -0.687635405318 * robotVelocity
                + 0.0310073919734 * robotVelocity2
                + -0.00100008820402 * robotVelocity3
                + -0.000116678815344 * robotVelocity4
                + 1.42324141334 * distance
                + -0.00539080634384 * distance * robotVelocity
                + -0.000237261757245 * distance * robotVelocity2
                + -4.59666269709e-06 * distance * robotVelocity3
                + -0.118718817623 * distance2
                + 0.000450591337966 * distance2 * robotVelocity
                + -3.72720592586e-05 * distance2 * robotVelocity2
                + 0.00705262442111 * distance3
                + -1.65383210993e-05 * distance3 * robotVelocity
                + -0.000163195976748 * distance4;
    }

    public static double calculateTimeOfFlight(double distance, double robotVelocity) {
        return 0.0; // TODO: add time of flight regression
    }

    public static double calculateAllowableShootAngleError(double distance, double robotVelocity) {
        return 0.0; // TODO: add shoot angle error regression
    }

    public static double calculateAllowableVelocityMagnitudeError(double distance, double robotVelocity) {
        return 0.0; // TODO: add velocity magnitude error regression
    }
}
