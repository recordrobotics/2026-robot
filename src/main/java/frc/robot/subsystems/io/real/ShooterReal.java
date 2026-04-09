package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ShooterIO;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.TalonFXMotorGroup;
import java.util.Arrays;

public class ShooterReal implements ShooterIO {

    private final TalonFXMotorGroup flywheelGroup;
    private final TalonFX hood;

    public ShooterReal() {
        flywheelGroup = new TalonFXMotorGroup(
                "Shooter",
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Shooter.FLYWHEEL_LEFT_ID, "Left", InvertedValue.CounterClockwise_Positive),
                new TalonFXMotorGroup.MotorConfig(
                        RobotMap.Shooter.FLYWHEEL_RIGHT_ID, "Right", InvertedValue.Clockwise_Positive));
        hood = new TalonFX(RobotMap.Shooter.HOOD_ID);
    }

    @Override
    public void applyFlywheelTalonFXConfig(TalonFXConfiguration configuration) {
        flywheelGroup.applyConfig(configuration);
    }

    @Override
    public void applyHoodTalonFXConfig(TalonFXConfiguration configuration) {
        hood.getConfigurator().apply(configuration);
    }

    @Override
    public void setFlywheelPositionMeters(double newValue) {
        flywheelGroup.setPosition(newValue);
    }

    @Override
    public void setHoodPositionRotations(double newValueRotations) {
        hood.setPosition(newValueRotations);
    }

    @Override
    public void setFlywheelControl(ControlRequest request) {
        flywheelGroup.setControl(request);
    }

    @Override
    public void setHoodControl(ControlRequest request) {
        hood.setControl(request);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelGroup.periodic();
        if (flywheelGroup.hasLostPosition()) { // position doesn't matter
            flywheelGroup.setPosition(0);
        }

        inputs.flywheelPositionMeters = flywheelGroup.getAveragePosition();
        inputs.flywheelVelocityMps = SimpleMath.average(flywheelGroup.getVelocities());
        inputs.flywheelVoltage = SimpleMath.average(flywheelGroup.getVoltages());
        inputs.flywheelCurrentDraw = Arrays.stream(flywheelGroup.getCurrents()).reduce(Amps.zero(), Current::plus);

        inputs.hoodConnected = hood.isConnected();
        inputs.hoodPositionRotations = hood.getPosition().getValueAsDouble();
        inputs.hoodVelocityRotationsPerSecond = hood.getVelocity().getValueAsDouble();
        inputs.hoodVoltage = hood.getMotorVoltage().getValueAsDouble();
        inputs.hoodCurrentDraw = hood.getSupplyCurrent().getValue();
    }

    @Override
    public void close() {
        flywheelGroup.close();
        hood.close();
    }
}
