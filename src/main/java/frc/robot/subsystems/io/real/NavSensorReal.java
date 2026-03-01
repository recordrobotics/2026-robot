package frc.robot.subsystems.io.real;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import frc.robot.subsystems.io.NavSensorIO;

public class NavSensorReal implements NavSensorIO {
    private final AHRS nav = new AHRS(NavXComType.kUSB1);
    private final boolean inverted;

    public NavSensorReal(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public void reset() {
        nav.reset();
    }

    @Override
    public void resetDisplacement() {
        nav.resetDisplacement();
    }

    @Override
    public double getAngle() {
        return inverted ? -nav.getAngle() : nav.getAngle();
    }

    @Override
    public double getPitch() {
        return nav.getPitch();
    }

    @Override
    public double getRoll() {
        return nav.getRoll();
    }

    @Override
    public double getYawRate() {
        return inverted ? -nav.getRate() : nav.getRate();
    }

    @Override
    public double getWorldLinearAccelX() {
        return nav.getWorldLinearAccelX();
    }

    @Override
    public double getWorldLinearAccelY() {
        return nav.getWorldLinearAccelY();
    }

    @Override
    public boolean isConnected() {
        return nav.isConnected();
    }

    @Override
    public void close() throws Exception {
        nav.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
