package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ImuIO;

public class ImuPigeon2 implements ImuIO {

    private final Pigeon2 pigeon;

    public ImuPigeon2() {
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
}
