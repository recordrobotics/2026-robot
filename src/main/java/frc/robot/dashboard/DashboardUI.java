package frc.robot.dashboard;

public final class DashboardUI {

    public static final AutonomousLayout Autonomous = new AutonomousLayout();
    public static final OverviewLayout Overview = new OverviewLayout();

    private DashboardUI() {}

    public static void update() {
        Autonomous.update();
        Overview.update();
    }
}
