package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.subsystems.io.NavSensorIO;

public class NavSensorNavX implements NavSensorIO {

    private static final double G = 9.81;

    private final AHRS nav = new AHRS(NavXComType.kUSB1);
    private final boolean inverted;

    public NavSensorNavX(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public void applyPigeon2Config(Pigeon2Configuration config) {
        /* not supported */
    }

    @Override
    public void reset() {
        nav.reset();
    }

    @Override
    public void resetDisplacement() {
        nav.resetDisplacement();
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(inverted ? -nav.getAngle() : nav.getAngle());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(nav.getPitch());
    }

    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(nav.getRoll());
    }

    @Override
    public AngularVelocity getYawRate() {
        return DegreesPerSecond.of(inverted ? -nav.getRate() : nav.getRate());
    }

    @Override
    public AngularVelocity getPitchRate() {
        return DegreesPerSecond.of(0);
    }

    @Override
    public AngularVelocity getRollRate() {
        return DegreesPerSecond.of(0);
    }

    @Override
    public LinearAcceleration getWorldLinearAccelX() {
        return MetersPerSecondPerSecond.of(G * nav.getWorldLinearAccelX());
    }

    @Override
    public LinearAcceleration getWorldLinearAccelY() {
        return MetersPerSecondPerSecond.of(G * nav.getWorldLinearAccelY());
    }

    @Override
    public boolean isConnected() {
        return nav.isConnected();
    }

    @Override
    public void close() throws Exception {
        nav.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
