package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.RobotModel.FuelManager.LineSegment;
import frc.robot.subsystems.io.FeederIO;

public class FeederSim implements FeederIO {

    private static final double SHOOT_VOLTAGE_MULTIPLIER = 0.6;

    private static final LineSegment topBeambreakLine = new LineSegment(
            new Translation3d(0.186525, 0.254875, 0.324841), new Translation3d(0.186525, -0.000875, 0.324841));
    private static final LineSegment bottomBeambreakLine = new LineSegment(
            new Translation3d(0.157081, 0.254875, 0.230972), new Translation3d(0.157081, -0.000875, 0.230972));

    private final double periodicDt;

    private final TalonFX feeder;
    private final DCMotor feederMotor = DCMotor.getKrakenX60(1);

    private final DCMotorSim feederSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(feederMotor, 0.0001243628, Constants.Feeder.GEAR_RATIO),
            feederMotor,
            0.0,
            0.0);

    private final DigitalInput bottomBeambreak = new DigitalInput(RobotMap.Feeder.BOTTOM_BEAM_BREAK_ID);
    private final DigitalInput topBeambreak = new DigitalInput(RobotMap.Feeder.TOP_BEAM_BREAK_ID);
    private final DIOSim bottomBeambreakSim;
    private final DIOSim topBeambreakSim;

    public FeederSim(double periodicDt) {
        this.periodicDt = periodicDt;

        feeder = new TalonFX(RobotMap.Feeder.MOTOR_ID);
        feeder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        feeder.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        bottomBeambreakSim = new DIOSim(bottomBeambreak);
        topBeambreakSim = new DIOSim(topBeambreak);
        bottomBeambreakSim.setIsInput(true);
        topBeambreakSim.setIsInput(true);

        RobotContainer.pdp.registerSimDevice(13, feeder.getSimState()::getSupplyCurrentMeasure);
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
        inputs.currentDraw = feeder.getSimState().getSupplyCurrentMeasure();

        inputs.bottomBeamBroken = !bottomBeambreak.get();
        inputs.topBeamBroken = !topBeambreak.get();
    }

    @Override
    public void simulationPeriodic() {
        feeder.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double feederVoltage = feeder.getSimState().getMotorVoltage();

        feederSimModel.setInputVoltage(
                (RobotContainer.model.fuelManager.isFuelInFeeder() ? SHOOT_VOLTAGE_MULTIPLIER : 1.0) * feederVoltage);
        feederSimModel.update(periodicDt);

        feeder.getSimState()
                .setRawRotorPosition(Constants.Feeder.GEAR_RATIO * feederSimModel.getAngularPositionRotations());
        feeder.getSimState()
                .setRotorVelocity(Constants.Feeder.GEAR_RATIO
                        * Units.radiansToRotations(feederSimModel.getAngularVelocityRadPerSec()));
        feeder.getSimState()
                .setRotorAcceleration(Constants.Feeder.GEAR_RATIO
                        * Units.radiansToRotations(feederSimModel.getAngularAccelerationRadPerSecSq()));

        bottomBeambreakSim.setValue(!RobotContainer.model.fuelManager.hasFuelIntersecting(bottomBeambreakLine));
        topBeambreakSim.setValue(!RobotContainer.model.fuelManager.hasFuelIntersecting(topBeambreakLine));
    }

    public boolean isOuttaking() {
        return feeder.getVelocity().getValueAsDouble() >= Constants.Feeder.INTAKE_VELOCITY_RPS / 2.0;
    }

    @Override
    public void close() {
        feeder.close();
        bottomBeambreak.close();
        topBeambreak.close();
    }
}
