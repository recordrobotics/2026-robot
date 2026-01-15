package frc.robot.subsystems.io.stub;

import frc.robot.subsystems.io.NavSensorIO;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class NavSensorStub implements NavSensorIO {

    @Override
    public void reset() {}

    @Override
    public void resetDisplacement() {}

    @Override
    public double getAngle() {
        return 0;
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }

    @Override
    public double getYawRate() {
        return 0;
    }

    @Override
    public double getWorldLinearAccelX() {
        return 0;
    }

    @Override
    public double getWorldLinearAccelY() {
        return 0;
    }

    @Override
    public boolean isConnected() {
        return false;
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void simulationPeriodic() {}
}
