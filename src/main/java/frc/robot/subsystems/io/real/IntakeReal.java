package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.TalonFXMotorGroup;
import java.util.Arrays;

public class IntakeReal implements IntakeIO {

    private final TalonFX wheel;
    private final TalonFXMotorGroup armGroup;

    public IntakeReal() {
        wheel = new TalonFX(RobotMap.Intake.WHEEL_ID);
        armGroup = new TalonFXMotorGroup(
                "Intake Arm",
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Intake.ARM_LEFT_ID, "Left", InvertedValue.CounterClockwise_Positive),
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Intake.ARM_RIGHT_ID, "Right", InvertedValue.Clockwise_Positive));
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        armGroup.applyConfig(configuration);
    }

    @Override
    public void applyWheelTalonFXConfig(TalonFXConfiguration configuration) {
        wheel.getConfigurator().apply(configuration);
    }

    @Override
    public void setArmControl(ControlRequest request) {
        armGroup.setControl(request);
    }

    @Override
    public void setWheelControl(ControlRequest request) {
        wheel.setControl(request);
    }

    @Override
    public void setArmPositionRotations(double newValue) {
        armGroup.setPosition(newValue);
    }

    @Override
    public void setWheelPositionMeters(double newValue) {
        wheel.setPosition(newValue);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        armGroup.periodic();

        inputs.armHasPosition = !armGroup.hasLostPosition();
        inputs.armPositionRotations = armGroup.getAveragePosition();
        inputs.armVelocityRotationsPerSecond = SimpleMath.average(armGroup.getVelocities());
        inputs.armVoltage = SimpleMath.average(armGroup.getVoltages());
        inputs.armCurrentDraw = Arrays.stream(armGroup.getCurrents()).reduce(Amps.zero(), Current::plus);

        inputs.wheelConnected = wheel.isConnected();
        inputs.wheelPositionMeters = wheel.getPosition().getValueAsDouble();
        inputs.wheelVelocityMps = wheel.getVelocity().getValueAsDouble();
        inputs.wheelVoltage = wheel.getMotorVoltage().getValueAsDouble();
        inputs.wheelCurrentDraw = wheel.getSupplyCurrent().getValue();
    }

    @Override
    public void close() {
        wheel.close();
        armGroup.close();
    }
}
