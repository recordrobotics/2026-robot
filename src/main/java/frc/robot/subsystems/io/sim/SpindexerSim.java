package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.SpindexerIO;

public class SpindexerSim implements SpindexerIO {

    private final double periodicDt;

    private final TalonFX spindexer;
    private final TalonFXSimState spindexerSimState;
    private final DCMotor spindexerMotor = DCMotor.getKrakenX60(1);

    private final DCMotorSim spindexerSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(spindexerMotor, 0.0028087817, Constants.Spindexer.GEAR_RATIO),
            spindexerMotor,
            0.0,
            0.0);

    public SpindexerSim(double periodicDt) {
        this.periodicDt = periodicDt;

        spindexer = new TalonFX(RobotMap.Spindexer.MOTOR_ID);
        spindexerSimState = spindexer.getSimState();

        spindexerSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        spindexerSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        spindexer.getConfigurator().apply(config);
    }

    @Override
    public void setMotionMagic(MotionMagicVelocityVoltage request) {
        spindexer.setControl(request);
    }

    @Override
    public void setVoltage(double newValue) {
        spindexer.setVoltage(newValue);
    }

    @Override
    public double getPositionRotations() {
        return spindexer.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return spindexer.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return spindexer.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getCurrentDrawAmps() {
        return spindexer.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        spindexerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double spindexerVoltage = spindexerSimState.getMotorVoltage();

        spindexerSimModel.setInputVoltage(spindexerVoltage);
        spindexerSimModel.update(periodicDt);

        spindexerSimState.setRawRotorPosition(
                Constants.Spindexer.GEAR_RATIO * spindexerSimModel.getAngularPositionRotations());
        spindexerSimState.setRotorVelocity(Constants.Spindexer.GEAR_RATIO
                * Units.radiansToRotations(spindexerSimModel.getAngularVelocityRadPerSec()));
        spindexerSimState.setRotorAcceleration(Constants.Spindexer.GEAR_RATIO
                * Units.radiansToRotations(spindexerSimModel.getAngularAccelerationRadPerSecSq()));
    }

    @Override
    public void close() {
        spindexer.close();
    }
}
