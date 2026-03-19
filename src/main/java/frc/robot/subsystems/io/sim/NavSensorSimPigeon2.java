package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.NavSensorIO;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class NavSensorSimPigeon2 implements NavSensorIO {

    private final GyroSimulation gyroSimulation;
    private final Pigeon2 pigeon;
    private final Pigeon2SimState pigeonSimState;

    public NavSensorSimPigeon2(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;

        this.pigeon = new Pigeon2(RobotMap.PIGEON_2_ID);
        this.pigeonSimState = pigeon.getSimState();
    }

    @Override
    public void applyPigeon2Config(Pigeon2Configuration config) {
        pigeon.getConfigurator().apply(config);
    }

    @Override
    public void reset() {
        gyroSimulation.setRotation(Rotation2d.kZero);
        pigeonSimState.setRawYaw(gyroSimulation.getGyroReading().getMeasure());
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
    public AngularVelocity getPitchRate() {
        return pigeon.getAngularVelocityYWorld().getValue();
    }

    @Override
    public AngularVelocity getRollRate() {
        return pigeon.getAngularVelocityXWorld().getValue();
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
        pigeonSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        pigeonSimState.setRawYaw(gyroSimulation.getGyroReading().getMeasure());
        pigeonSimState.setPitch(0);
        pigeonSimState.setRoll(0);
        pigeonSimState.setAngularVelocityX(0);
        pigeonSimState.setAngularVelocityY(0);
        pigeonSimState.setAngularVelocityZ(gyroSimulation.getMeasuredAngularVelocity());
    }
}
