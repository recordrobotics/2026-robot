package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.dashboard.DashboardUI;

public class VibrateXbox extends Command {

    private RumbleType type;
    private double value;

    public VibrateXbox(RumbleType type, double value) {
        this.type = type;
        this.value = value;
    }

    @Override
    public void initialize() {
        DashboardUI.Overview.getControl().vibrate(type, value);
    }

    @Override
    public void end(boolean interrupted) {
        DashboardUI.Overview.getControl().vibrate(RumbleType.kBothRumble, 0);
    }
}
