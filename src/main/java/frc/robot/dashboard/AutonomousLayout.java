package frc.robot.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.commands.auto.CreateAutoRoutineException;
import frc.robot.commands.auto.IAutoRoutine;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.libraries.Elastic;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@SuppressWarnings("java:S2325")
public final class AutonomousLayout extends AbstractLayout {

    private final Field2d field = new Field2d();
    private LoggedDashboardChooser<Command> autoChooser;
    private LoggedDashboardChooser<FieldStartingLocation> fieldStartingLocationChooser =
            new LoggedDashboardChooser<>("Starting Location");

    public AutonomousLayout() {
        buildSendable("Field", field);

        EnumSet.allOf(FieldStartingLocation.class)
                .forEach(v -> fieldStartingLocationChooser.addOption(v.toString(), v));
        fieldStartingLocationChooser.addDefaultOption(
                FieldStartingLocation.BARGE_CENTER.toString(), FieldStartingLocation.BARGE_CENTER);

        SmartDashboard.putBoolean("Autonomous/ResetLocationButton", false);
        SmartDashboard.putBoolean("Autonomous/LimelightRotation", false);
        SmartDashboard.putBoolean("Autonomous/EncoderReset", false);
        SmartDashboard.putBoolean("Autonomous/ForceMT1", false);
    }

    /**
     * Sets up the auto chooser with the given routines in addition to pathplanner autos loaded by default.
     * @param routines the auto routines to add to the chooser
     */
    @SafeVarargs
    public final void setupAutoChooser(final Supplier<IAutoRoutine>... routines) {
        autoChooser = new LoggedDashboardChooser<>("Auto Code", AutoBuilder.buildAutoChooser());

        // Add non-pathplanner autos
        for (Supplier<IAutoRoutine> routineSupplier : routines) {

            IAutoRoutine routine;
            try {
                routine = routineSupplier.get();
            } catch (CreateAutoRoutineException e) {
                ConsoleLogger.logError("Can't get auto routine", e);
                continue;
            }

            if (routine instanceof Command cmd) {
                autoChooser.addOption(routine.getAutoName(), cmd);
            } else {
                throw new IllegalArgumentException("Auto routine does not implement Command: " + routine.getAutoName());
            }
        }
    }

    public void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void setVisionPose(String name, Pose2d pose) {
        field.getObject(name).setPose(pose);
    }

    @Override
    public void switchTo() {
        Elastic.selectTab("Autonomous");
    }

    @Override
    protected NetworkTable getNetworkTable() {
        return NetworkTableInstance.getDefault().getTable("/SmartDashboard/Autonomous");
    }

    public Command getAutoChooser() {
        return autoChooser.get();
    }

    public FieldStartingLocation getStartingLocation() {
        return fieldStartingLocationChooser.get() == null
                ? FieldStartingLocation.BARGE_CENTER
                : fieldStartingLocationChooser.get();
    }

    public boolean isResetLocationPressed() {
        return SmartDashboard.getBoolean("Autonomous/ResetLocationButton", false);
    }

    public boolean isLimelightRotationPressed() {
        return SmartDashboard.getBoolean("Autonomous/LimelightRotation", false);
    }

    public boolean isEncoderResetPressed() {
        return SmartDashboard.getBoolean("Autonomous/EncoderReset", false);
    }

    public boolean isForceMT1Pressed() {
        return SmartDashboard.getBoolean("Autonomous/ForceMT1", false);
    }
}
