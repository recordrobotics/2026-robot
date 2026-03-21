package frc.robot.subsystems.shootorchestrator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubRegressionCalculator implements ShotCalculator {

    private static final double FLYWHEEL_TO_FUEL_RATIO = 0.385;

    static {
        SmartDashboard.putNumber("FLYWHEEL_RATIO", FLYWHEEL_TO_FUEL_RATIO);
    }

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
        return 95.0476215151
                + 7.22781779628 * robotVelocity
                + 0.446187166766 * robotVelocity2
                + -0.143025576382 * robotVelocity3
                + -0.0178376322206 * robotVelocity4
                + -25.4495442603 * distance
                + 0.27772525426 * distance * robotVelocity
                + 0.0236510086091 * distance * robotVelocity2
                + 0.0194862031565 * distance * robotVelocity3
                + 6.92921040811 * distance2
                + -0.186088818887 * distance2 * robotVelocity
                + -0.00120770883884 * distance2 * robotVelocity2
                + -0.89854631409 * distance3
                + 0.0108330719224 * distance3 * robotVelocity
                + 0.0416242296937 * distance4;
    }

    public static double calculateBallVelocity(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 6.36807950256
                + -0.318415367375 * robotVelocity
                + 0.0568602191061 * robotVelocity2
                + -0.00155522629749 * robotVelocity3
                + -0.000626428266815 * robotVelocity4
                + -0.791430248818 * distance
                + -0.119230385626 * distance * robotVelocity
                + -0.0024168567017 * distance * robotVelocity2
                + 0.000168717081825 * distance * robotVelocity3
                + 0.51592576216 * distance2
                + 0.0212946084093 * distance2 * robotVelocity
                + -5.69927539992e-05 * distance2 * robotVelocity2
                + -0.0741889239284 * distance3
                + -0.00147749738957 * distance3 * robotVelocity
                + 0.00363337018461 * distance4;
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
