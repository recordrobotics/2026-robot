package frc.robot.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.control.AbstractControl;
import frc.robot.utils.libraries.Elastic;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class OverviewLayout extends AbstractLayout {

    public enum DriverOrientation {
        X_AXIS_TOWARDS_TRIGGER("Competition"),
        Y_AXIS_TOWARDS_TRIGGER("Y Axis"),
        X_AXIS_INVERTED_TOWARDS_TRIGGER("Couch Drive");

        public final String displayName;

        DriverOrientation(String orientation) {
            displayName = orientation;
        }
    }

    private LoggedDashboardChooser<DriverOrientation> driverOrientation =
            new LoggedDashboardChooser<>("Driver Orientation");
    private LoggedDashboardChooser<AbstractControl> driveMode = new LoggedDashboardChooser<>("Drive Mode");
    private AbstractControl defaultControl;
    private AbstractControl testControl;

    private Supplier<Boolean> navSensorValue = () -> false;

    public OverviewLayout() {
        addValueSendable("Nav Sensor", () -> navSensorValue.get(), "boolean");
    }

    /**
     * Initializes the control object
     *
     * @param defaultControl the first term will always be the default control object
     * @param controls any other control objects you want to initialize
     */
    public void addControls(AbstractControl defaultControl, AbstractControl... controls) {
        this.defaultControl = defaultControl;

        // Sets up drive mode options
        for (AbstractControl abstractControl : controls) {
            driveMode.addOption(abstractControl.getClass().getSimpleName(), abstractControl);
        }
        driveMode.addDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);

        EnumSet.allOf(DriverOrientation.class).forEach(v -> driverOrientation.addOption(v.displayName, v));
        driverOrientation.addDefaultOption(
                DriverOrientation.X_AXIS_TOWARDS_TRIGGER.displayName, DriverOrientation.X_AXIS_TOWARDS_TRIGGER);
    }

    public void setNavSensor(Supplier<Boolean> navSensor) {
        navSensorValue = navSensor;
    }

    @Override
    public void switchTo() {
        Elastic.selectTab("Overview");
    }

    @Override
    protected NetworkTable getNetworkTable() {
        return NetworkTableInstance.getDefault().getTable("/SmartDashboard/Overview");
    }

    public DriverOrientation getDriverOrientation() {
        return driverOrientation.get();
    }

    public void setTestControl(AbstractControl testControl) {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.TEST) return;
        this.testControl = testControl;
    }

    public AbstractControl getControl() {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.TEST) {
            if (testControl == null) {
                throw new IllegalStateException("Test control is not set!");
            }
            return testControl;
        }

        if (driveMode.get() == null) return defaultControl;
        return driveMode.get();
    }
}
