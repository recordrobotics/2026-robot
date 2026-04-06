package frc.robot.subsystems.io.stub;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import edu.wpi.first.units.measure.Current;
import frc.robot.subsystems.io.SpindexerIO;

public class SpindexerStub implements SpindexerIO {

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration config) {
        // stub
    }

    @Override
    public void setMotionMagic(MotionMagicVelocityVoltage request) {
        // stub
    }

    @Override
    public void setVoltage(double newValue) {
        // stub
    }

    @Override
    public double getPositionRotations() {
        // stub
        return 0.0;
    }

    @Override
    public double getVelocityRotationsPerSecond() {
        // stub
        return 0.0;
    }

    @Override
    public double getVoltage() {
        // stub
        return 0.0;
    }

    @Override
    public Current getCurrentDraw() {
        // stub
        return Amps.zero();
    }

    @Override
    public void simulationPeriodic() {
        // stub
    }

    @Override
    public void close() {
        // stub
    }
}
