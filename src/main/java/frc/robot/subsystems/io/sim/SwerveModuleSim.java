package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class SwerveModuleSim implements SwerveModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final DutyCycleEncoder absoluteTurningMotorEncoder;

    private final TalonFXSimState driveMotorSim;
    private final TalonFXSimState turningMotorSim;
    private final SimDouble absoluteTurningMotorEncoderPosition;

    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        public final int id;

        private final TalonFXSimState talonFXSimState;

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
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    public SwerveModuleSim(SwerveModuleSimulation moduleSimulation, ModuleConstants m) {
        driveMotor = new TalonFX(m.driveMotorChannel());
        turningMotor = new TalonFX(m.turningMotorChannel());

        absoluteTurningMotorEncoder = new DutyCycleEncoder(m.absoluteTurningMotorEncoderChannel());
        SimDeviceSim absoluteTurningMotorEncoderSim =
                new SimDeviceSim("DutyCycle:DutyCycleEncoder", m.absoluteTurningMotorEncoderChannel());
        absoluteTurningMotorEncoderPosition = absoluteTurningMotorEncoderSim.getDouble("Position");

        driveMotorSim = driveMotor.getSimState();
        turningMotorSim = turningMotor.getSimState();

        turningMotorSim.Orientation = ChassisReference.Clockwise_Positive;

        moduleSimulation.useDriveMotorController(new TalonFXMotorControllerSim(driveMotor));
        moduleSimulation.useSteerMotorController(new TalonFXMotorControllerSim(turningMotor));
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
    public void setDriveMotorVoltage(double newValue) {
        driveMotor.setVoltage(newValue);
    }

    @Override
    public void setTurnMotorVoltage(double newValue) {
        turningMotor.setVoltage(newValue);
    }

    @Override
    public void setTurnMotorMotionMagic(MotionMagicExpoVoltage request) {
        turningMotor.setControl(request);
    }

    @Override
    public void setDriveMotorMotionMagic(MotionMagicVelocityVoltage request) {
        driveMotor.setControl(request);
    }

    @Override
    public double getDriveMotorVoltage() {
        return driveMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getTurnMotorVoltage() {
        return turningMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setDriveMotorPercent(double newValue) {
        driveMotor.set(newValue);
    }

    @Override
    public void setTurnMotorPercent(double newValue) {
        turningMotor.set(newValue);
    }

    @Override
    public double getDriveMotorPercent() {
        return driveMotor.get();
    }

    @Override
    public double getTurnMotorPercent() {
        return turningMotor.get();
    }

    @Override
    public double getAbsoluteEncoder() {
        return absoluteTurningMotorEncoder.get();
    }

    @Override
    public double getTurnMechanismPosition() {
        return turningMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getTurnMechanismVelocity() {
        return turningMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getDriveMechanismPosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getDriveMechanismVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getDriveMechanismAcceleration() {
        return driveMotor.getAcceleration().getValueAsDouble();
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
    public double getDriveMotorCurrentDrawAmps() {
        if (driveMotorSim.Orientation == ChassisReference.Clockwise_Positive)
            return driveMotor.getSupplyCurrent().getValueAsDouble();
        else return -driveMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getTurnMotorCurrentDrawAmps() {
        if (turningMotorSim.Orientation == ChassisReference.Clockwise_Positive)
            return turningMotor.getSupplyCurrent().getValueAsDouble();
        else return -turningMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        driveMotor.close();
        turningMotor.close();
        absoluteTurningMotorEncoder.close();
    }

    @Override
    public void simulationPeriodic() {
        absoluteTurningMotorEncoderPosition.set(getTurnMechanismPosition());
    }
}
