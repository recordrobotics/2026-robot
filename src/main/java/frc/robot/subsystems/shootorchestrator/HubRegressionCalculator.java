package frc.robot.subsystems.shootorchestrator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubRegressionCalculator implements ShotCalculator {

    private static final double FLYWHEEL_TO_FUEL_RATIO = 0.44;

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
        return 93.4876402699
                + 7.50657522373 * robotVelocity
                + 0.836207774564 * robotVelocity2
                + -0.0580081295263 * robotVelocity3
                + -0.00865669534907 * robotVelocity4
                + -25.9129470879 * distance
                + -0.366647509675 * distance * robotVelocity
                + -0.218161409577 * distance * robotVelocity2
                + 0.00938902627082 * distance * robotVelocity3
                + 7.79974599763 * distance2
                + -0.131863606838 * distance2 * robotVelocity
                + 0.0212856707222 * distance2 * robotVelocity2
                + -1.13272814726 * distance3
                + 0.0129857323031 * distance3 * robotVelocity
                + 0.0593987888648 * distance4;
    }

    public static double calculateBallVelocity(double distance, double robotVelocity) {
        double distance2 = distance * distance;
        double distance3 = distance2 * distance;
        double distance4 = distance3 * distance;
        double robotVelocity2 = robotVelocity * robotVelocity;
        double robotVelocity3 = robotVelocity2 * robotVelocity;
        double robotVelocity4 = robotVelocity3 * robotVelocity;
        return 5.74012474728
                + -0.539058185805 * robotVelocity
                + 0.0820413980332 * robotVelocity2
                + 0.00171418522627 * robotVelocity3
                + -0.000528210655164 * robotVelocity4
                + -0.173086182149 * distance
                + 0.0451770736047 * distance * robotVelocity
                + -0.0135844476726 * distance * robotVelocity2
                + -0.000321450063114 * distance * robotVelocity3
                + 0.31693651342 * distance2
                + -0.0193555976086 * distance2 * robotVelocity
                + 0.000879636893034 * distance2 * robotVelocity2
                + -0.0461698678218 * distance3
                + 0.00155733901075 * distance3 * robotVelocity
                + 0.00229893444873 * distance4;
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
