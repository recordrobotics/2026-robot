package frc.robot.subsystems.shootorchestrator;

import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class SimpleParabolaCalculator implements ShotCalculator {

    @Override
    public double fuelToFlywheelVelocity(double fuelVelocityMps) {
        return fuelVelocityMps / 0.8;
    }

    @Override
    public double flywheelToFuelVelocity(double flywheelVelocityMps) {
        return flywheelVelocityMps * 0.8;
    }

    @Override
    public ShotCalculation calculateShot(double distanceToTargetMeters, double robotVelocityTowardsTargetMps) {

        double targetHeight = Units.feetToMeters(6);
        double maxHeight = Units.feetToMeters(8);
        double height = maxHeight - 0.4;

        double timeOfFlight = (Math.sqrt(2 * GamePieceProjectile.GRAVITY * height)
                        + Math.sqrt(2 * GamePieceProjectile.GRAVITY * (maxHeight - targetHeight)))
                / GamePieceProjectile.GRAVITY;
        double velocity = Math.sqrt((distanceToTargetMeters * distanceToTargetMeters) / (timeOfFlight * timeOfFlight)
                + 2 * GamePieceProjectile.GRAVITY * height);
        double angle =
                Math.atan(timeOfFlight * Math.sqrt(2 * GamePieceProjectile.GRAVITY * height) / distanceToTargetMeters);

        if (Double.isNaN(velocity) || Double.isNaN(angle)) {
            return new ShotCalculation(0, 0, 0, 0, 0);
        }
        return new ShotCalculation(angle, velocity, timeOfFlight, 0, 0);
    }
}
