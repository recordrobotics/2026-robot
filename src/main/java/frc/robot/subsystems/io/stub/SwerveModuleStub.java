package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

@SuppressWarnings("java:S1186") // Methods intentionally left blank
public class SwerveModuleStub implements SwerveModuleIO {
    public SwerveModuleStub(ModuleConstants m, int driveTrack, int turnTrack) {}

    public SwerveModuleStub(
            SwerveModuleSimulation moduleSimulation,
            ModuleConstants m,
            int driveTrack,
            int turnTrack,
            int drivePdpChannel,
            int turnPdpChannel) {}

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {}

    @Override
    public void applyDriveTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void applyTurnTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void applyTurningEncoderConfig(CANcoderConfiguration configuration) {}

    @Override
    public void setTurnControl(ControlRequest request) {}

    @Override
    public void setDriveControl(ControlRequest request) {}

    @Override
    public void setDriveMechanismPosition(double newValue) {}

    @Override
    public void setTurnMechanismPosition(double newValue) {}

    @Override
    public void close() {}

    @Override
    public void simulationPeriodic() {}
}
