package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.FeederIO;
import frc.robot.utils.TalonFXOrchestra;

public class FeederReal implements FeederIO {

    private final TalonFX feeder;
    private final DigitalInput bottomBeambreak = new DigitalInput(RobotMap.Feeder.BOTTOM_BEAM_BREAK_ID);
    private final DigitalInput topBeambreak = new DigitalInput(RobotMap.Feeder.TOP_BEAM_BREAK_ID);

    public FeederReal() {
        feeder = new TalonFX(RobotMap.Feeder.MOTOR_ID);
        RobotContainer.orchestra.add(feeder, TalonFXOrchestra.Tracks.FEEDER);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        feeder.getConfigurator().apply(config);
    }

    @Override
    public void setControl(ControlRequest request) {
        feeder.setControl(request);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.connected = feeder.isConnected();
        inputs.positionRotations = feeder.getPosition().getValueAsDouble();
        inputs.velocityRotationsPerSecond = feeder.getVelocity().getValueAsDouble();
        inputs.voltage = feeder.getMotorVoltage().getValueAsDouble();
        inputs.currentDraw = feeder.getSupplyCurrent().getValue();

        inputs.bottomBeamBroken = !bottomBeambreak.get();
        inputs.topBeamBroken = !topBeambreak.get();
    }

    @Override
    public void close() {
        feeder.close();
        bottomBeambreak.close();
        topBeambreak.close();
    }
}
