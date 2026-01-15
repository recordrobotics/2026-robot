package frc.robot.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

@SuppressWarnings("java:S109")
public final class DCMotors {
    private DCMotors() {}

    /**
     * Return a gearbox of Kraken X44 brushless motors.
     *
     * @param numMotors Number of motors in the gearbox.
     * @return a gearbox of Kraken X44 motors.
     */
    public static DCMotor getKrakenX44(int numMotors) {
        // From https://store.ctr-electronics.com/announcing-kraken-x44/
        return new DCMotor(12, 4.05, 275, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530), numMotors);
    }
}
