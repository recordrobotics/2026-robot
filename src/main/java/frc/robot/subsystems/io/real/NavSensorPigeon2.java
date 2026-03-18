package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.NavSensorIO;

public class NavSensorPigeon2 implements NavSensorIO {

    private final Pigeon2 pigeon;

    public NavSensorPigeon2() {
        this.pigeon = new Pigeon2(RobotMap.PIGEON_2_ID);
    }

    @Override
    public void applyPigeon2Config(Pigeon2Configuration config) {
        pigeon.getConfigurator().apply(config);
    }

    @Override
    public void reset() {
        pigeon.reset();
    }

    @Override
    public void resetDisplacement() {
        /* not supported */
    }

    @Override
    public Rotation2d getYaw() {
        return new Rotation2d(pigeon.getYaw().getValue());
    }

    @Override
    public Rotation2d getPitch() {
        return new Rotation2d(pigeon.getPitch().getValue());
    }

    @Override
    public Rotation2d getRoll() {
        return new Rotation2d(pigeon.getRoll().getValue());
    }

    @Override
    public AngularVelocity getYawRate() {
        return pigeon.getAngularVelocityZWorld().getValue();
    }

    @Override
    public LinearAcceleration getWorldLinearAccelX() {
        return pigeon.getAccelerationX().getValue();
    }

    @Override
    public LinearAcceleration getWorldLinearAccelY() {
        return pigeon.getAccelerationY().getValue();
    }

    @Override
    public boolean isConnected() {
        return pigeon.isConnected();
    }

    @Override
    public void close() throws Exception {
        pigeon.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
