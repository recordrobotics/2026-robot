package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.TurretIO;
import frc.robot.utils.TalonFXOrchestra;

public class TurretReal implements TurretIO {

    private final TalonFX turret;

    private final DigitalInput frontLeftLimitSwitch = new DigitalInput(RobotMap.Turret.FRONT_LEFT_LIMIT_SWITCH_ID);
    private final DigitalInput backLeftLimitSwitch = new DigitalInput(RobotMap.Turret.BACK_LEFT_LIMIT_SWITCH_ID);
    private final DigitalInput backRightLimitSwitch = new DigitalInput(RobotMap.Turret.BACK_RIGHT_LIMIT_SWITCH_ID);

    public TurretReal() {
        turret = new TalonFX(RobotMap.Turret.MOTOR_ID);
        RobotContainer.orchestra.add(turret, TalonFXOrchestra.Tracks.TURRET);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        turret.getConfigurator().apply(configuration);
    }

    @Override
    public void setControl(ControlRequest request) {
        turret.setControl(request);
    }

    @Override
    public void setPositionRotations(double newValue) {
        turret.setPosition(newValue);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = turret.isConnected();
        inputs.positionRotations = turret.getPosition().getValueAsDouble();
        inputs.velocityRotationsPerSecond = turret.getVelocity().getValueAsDouble();
        inputs.voltage = turret.getMotorVoltage().getValueAsDouble();
        inputs.currentDraw = turret.getSupplyCurrent().getValue();

        inputs.forwardSoftLimitHit =
                turret.getFault_ForwardSoftLimit().getValue().booleanValue();
        inputs.reverseSoftLimitHit =
                turret.getFault_ReverseSoftLimit().getValue().booleanValue();

        inputs.limitSwitchStates = new LimitSwitchStates(
                !frontLeftLimitSwitch.get(), !backLeftLimitSwitch.get(), !backRightLimitSwitch.get());
    }

    @Override
    public void close() {
        turret.close();
        frontLeftLimitSwitch.close();
        backLeftLimitSwitch.close();
        backRightLimitSwitch.close();
    }
}
