package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionPanel extends SubsystemBase implements AutoCloseable {
    private static final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

    /** frees up all hardware allocations */
    @Override
    public void close() {
        pdp.close();
    }
}
