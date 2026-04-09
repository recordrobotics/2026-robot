package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.Milliamps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class SwerveModuleSim implements SwerveModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder absoluteTurningMotorEncoder;

    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        public final int id;

        private final TalonFXSimState talonFXSimState;

        private AngularVelocity lastVelocity = RotationsPerSecond.of(0);

        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = talonFX.getDeviceID();
            this.talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setRotorAcceleration(
                    (encoderVelocity.in(RotationsPerSecond) - lastVelocity.in(RotationsPerSecond))
                            / SimulatedArena.getSimulationDt().in(Seconds));
            lastVelocity = encoderVelocity;
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    public SwerveModuleSim(
            SwerveModuleSimulation moduleSimulation, ModuleConstants m, int drivePdpChannel, int turnPdpChannel) {
        driveMotor = new TalonFX(m.driveMotorChannel());
        turningMotor = new TalonFX(m.turningMotorChannel());
        absoluteTurningMotorEncoder = new CANcoder(m.absoluteTurningMotorEncoderChannel());

        driveMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        turningMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;

        driveMotor.getSimState().setMotorType(MotorType.KrakenX60);
        turningMotor.getSimState().setMotorType(MotorType.KrakenX44);

        absoluteTurningMotorEncoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        absoluteTurningMotorEncoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Green);
        absoluteTurningMotorEncoder.getSimState().SensorOffset = m.turningEncoderOffset();
        moduleSimulation.useDriveMotorController(new TalonFXMotorControllerSim(driveMotor));
        moduleSimulation.useSteerMotorController(new TalonFXMotorControllerSim(turningMotor));

        RobotContainer.pdp.registerSimDevice(drivePdpChannel, driveMotor.getSimState()::getSupplyCurrentMeasure);
        RobotContainer.pdp.registerSimDevice(turnPdpChannel, turningMotor.getSimState()::getSupplyCurrentMeasure);
        RobotContainer.pdp.registerSimMiniPdpDevice(() -> Milliamps.of(50));
    }

    @Override
    public void applyDriveTalonFXConfig(TalonFXConfiguration configuration) {
        driveMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void applyTurnTalonFXConfig(TalonFXConfiguration configuration) {
        turningMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void applyTurningEncoderConfig(CANcoderConfiguration configuration) {
        absoluteTurningMotorEncoder.getConfigurator().apply(configuration);
    }

    @Override
    public void setDriveMechanismPosition(double newValue) {
        driveMotor.setPosition(newValue);
    }

    @Override
    public void setTurnMechanismPosition(double newValue) {
        /* don't reset position in simulation */
    }

    @Override
    public void setDriveControl(ControlRequest request) {
        driveMotor.setControl(request);
    }

    @Override
    public void setTurnControl(ControlRequest request) {
        turningMotor.setControl(request);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorConnected = driveMotor.isConnected();
        inputs.driveMotorPositionMeters = driveMotor.getPosition().getValueAsDouble();
        inputs.driveMotorVelocityMps = driveMotor.getVelocity().getValueAsDouble();
        inputs.driveMotorAccelerationMps2 = driveMotor.getAcceleration().getValueAsDouble();
        inputs.driveMotorVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveMotorCurrentDraw = driveMotor.getSimState().getSupplyCurrentMeasure();

        inputs.turnMotorConnected = turningMotor.isConnected();
        inputs.turnMotorPositionRotations = turningMotor.getPosition().getValueAsDouble();
        inputs.turnMotorVelocityRps = turningMotor.getVelocity().getValueAsDouble();
        inputs.turnMotorVoltage = turningMotor.getMotorVoltage().getValueAsDouble();
        inputs.turnMotorCurrentDraw = turningMotor.getSimState().getSupplyCurrentMeasure();

        inputs.encoderConnected = absoluteTurningMotorEncoder.isConnected();
        inputs.encoderPositionRotations =
                absoluteTurningMotorEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.encoderMagnetHealth =
                absoluteTurningMotorEncoder.getMagnetHealth().getValue();
    }

    @Override
    public void close() {
        driveMotor.close();
        turningMotor.close();
        absoluteTurningMotorEncoder.close();
    }

    @Override
    public void simulationPeriodic() {
        absoluteTurningMotorEncoder.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        absoluteTurningMotorEncoder
                .getSimState()
                .setRawPosition(turningMotor.getPosition().getValueAsDouble());
        absoluteTurningMotorEncoder
                .getSimState()
                .setVelocity(turningMotor.getVelocity().getValueAsDouble());
    }
}
