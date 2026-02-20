package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.FeederIO;

public class FeederSim implements FeederIO {

    private final double periodicDt;

    private final TalonFX feeder;
    private final TalonFXSimState feederSimState;
    private final DCMotor feederMotor = DCMotor.getKrakenX60(1);

    private final DCMotorSim feederSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(feederMotor, 0.0001243628, Constants.Feeder.GEAR_RATIO),
            feederMotor,
            0.0,
            0.0);

    private final DigitalInput bottomBeambreak = new DigitalInput(RobotMap.Feeder.BOTTOM_BEAM_BREAK_ID);
    private final DigitalInput topBeambreak = new DigitalInput(RobotMap.Feeder.TOP_BEAM_BREAK_ID);
    private final SimDevice bottomBeambreakSim = SimDevice.create("DigitalInput", RobotMap.Feeder.BOTTOM_BEAM_BREAK_ID);
    private final SimDevice topBeambreakSim = SimDevice.create("DigitalInput", RobotMap.Feeder.TOP_BEAM_BREAK_ID);
    private final SimBoolean bottomBeambreakSimValue;
    private final SimBoolean topBeambreakSimValue;

    public FeederSim(double periodicDt) {
        this.periodicDt = periodicDt;

        feeder = new TalonFX(RobotMap.Feeder.MOTOR_ID);
        feederSimState = feeder.getSimState();
        feederSimState.Orientation = ChassisReference.Clockwise_Positive;
        feederSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        if (bottomBeambreakSim != null)
            bottomBeambreakSimValue = bottomBeambreakSim.createBoolean("Value", Direction.kOutput, true);
        else bottomBeambreakSimValue = null;

        if (bottomBeambreakSim != null) bottomBeambreak.setSimDevice(bottomBeambreakSim);
        else bottomBeambreak.close();

        if (topBeambreakSim != null)
            topBeambreakSimValue = topBeambreakSim.createBoolean("Value", Direction.kOutput, true);
        else topBeambreakSimValue = null;

        if (topBeambreakSim != null) topBeambreak.setSimDevice(topBeambreakSim);
        else topBeambreak.close();
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        feeder.getConfigurator().apply(config);
    }

    @Override
    public void setMotionMagic(MotionMagicVelocityVoltage request) {
        feeder.setControl(request);
    }

    @Override
    public void setVoltage(double newValue) {
        feeder.setVoltage(newValue);
    }

    @Override
    public double getPositionRotations() {
        return feeder.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        return feeder.getVelocity().getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return feeder.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getCurrentDrawAmps() {
        return feeder.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public boolean isBottomBeamBroken() {
        if (bottomBeambreakSim != null) return !bottomBeambreakSimValue.get();
        else return false;
    }

    @Override
    public boolean isTopBeamBroken() {
        if (topBeambreakSim != null) return !topBeambreakSimValue.get();
        else return false;
    }

    public void setBottomBeamBroken(boolean newValue) {
        if (bottomBeambreakSimValue != null) bottomBeambreakSimValue.set(!newValue);
    }

    public void setTopBeamBroken(boolean newValue) {
        if (topBeambreakSimValue != null) topBeambreakSimValue.set(!newValue);
    }

    @Override
    public void simulationPeriodic() {
        feederSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double feederVoltage = feederSimState.getMotorVoltage();

        feederSimModel.setInputVoltage(feederVoltage);
        feederSimModel.update(periodicDt);

        feederSimState.setRawRotorPosition(Constants.Feeder.GEAR_RATIO * feederSimModel.getAngularPositionRotations());
        feederSimState.setRotorVelocity(
                Constants.Feeder.GEAR_RATIO * Units.radiansToRotations(feederSimModel.getAngularVelocityRadPerSec()));
        feederSimState.setRotorAcceleration(Constants.Feeder.GEAR_RATIO
                * Units.radiansToRotations(feederSimModel.getAngularAccelerationRadPerSecSq()));
    }

    @Override
    public void close() {
        feeder.close();
        if (bottomBeambreakSim != null) {
            bottomBeambreakSim.close();
            bottomBeambreak.close();
        }
        if (topBeambreakSim != null) {
            topBeambreakSim.close();
            topBeambreak.close();
        }
    }
}
