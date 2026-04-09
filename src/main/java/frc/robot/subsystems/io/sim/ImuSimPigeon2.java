package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.Milliamps;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ImuIO;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class ImuSimPigeon2 implements ImuIO {

    private final GyroSimulation gyroSimulation;
    private final Pigeon2 pigeon;
    private final Pigeon2SimState pigeonSimState;

    public ImuSimPigeon2(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;

        this.pigeon = new Pigeon2(RobotMap.PIGEON_2_ID);
        this.pigeonSimState = pigeon.getSimState();

        RobotContainer.pdp.registerSimMiniPdpDevice(() -> Milliamps.of(40));
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
    public void updateInputs(ImuIOInputs inputs) {
        inputs.connected = pigeon.isConnected();
        inputs.yaw = new Rotation2d(pigeon.getYaw().getValue());
        inputs.pitch = new Rotation2d(pigeon.getPitch().getValue());
        inputs.roll = new Rotation2d(pigeon.getRoll().getValue());
        inputs.yawRate = pigeon.getAngularVelocityZWorld().getValue();
        inputs.pitchRate = pigeon.getAngularVelocityYWorld().getValue();
        inputs.rollRate = pigeon.getAngularVelocityXWorld().getValue();
        inputs.worldLinearAccelX = pigeon.getAccelerationX().getValue();
        inputs.worldLinearAccelY = pigeon.getAccelerationY().getValue();
    }

    @Override
    public void close() {
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
