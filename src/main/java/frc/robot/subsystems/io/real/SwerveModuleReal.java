package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;

public class SwerveModuleReal implements SwerveModuleIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final DutyCycleEncoder absoluteTurningMotorEncoder;

    public SwerveModuleReal(double periodicDt, ModuleConstants m) {
        this.periodicDt = periodicDt;

        driveMotor = new TalonFX(m.driveMotorChannel());
        turningMotor = new TalonFX(m.turningMotorChannel());

        absoluteTurningMotorEncoder = new DutyCycleEncoder(m.absoluteTurningMotorEncoderChannel());
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
        turningMotor.setPosition(newValue);
    }

    @Override
    public void close() throws Exception {
        driveMotor.close();
        turningMotor.close();
        absoluteTurningMotorEncoder.close();
    }

    @Override
    public double getDriveMotorCurrentDrawAmps() {
        return driveMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getTurnMotorCurrentDrawAmps() {
        return turningMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
