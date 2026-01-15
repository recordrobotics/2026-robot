package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class HumanPlayerSimulation extends SubsystemBase {

    public enum HumanPlayerSimulationStrategy {
        NONE,
        ONLY_GROUND_NEAR,
        ONLY_GROUND_FAR,
        ONLY_SOURCE,
        GROUND_WHEN_OCCUPIED,
    }

    public enum HumanPlayerSource {
        RED_LEFT,
        RED_RIGHT,
        BLUE_LEFT,
        BLUE_RIGHT
    }

    private LoggedDashboardChooser<HumanPlayerSimulationStrategy> humanPlayerStrategyChooser;
    private List<HumanPlayer> humanPlayers = new ArrayList<>();

    public HumanPlayerSimulation() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            humanPlayerStrategyChooser = new LoggedDashboardChooser<>("HumanPlayerStrategy");
            EnumSet.allOf(HumanPlayerSimulationStrategy.class)
                    .forEach(v -> humanPlayerStrategyChooser.addOption(v.name(), v));
            humanPlayerStrategyChooser.addDefaultOption(
                    HumanPlayerSimulationStrategy.GROUND_WHEN_OCCUPIED.toString(),
                    HumanPlayerSimulationStrategy.GROUND_WHEN_OCCUPIED);

            humanPlayers.add(new HumanPlayer(humanPlayerStrategyChooser.get(), HumanPlayerSource.RED_LEFT));
            humanPlayers.add(new HumanPlayer(humanPlayerStrategyChooser.get(), HumanPlayerSource.RED_RIGHT));
            humanPlayers.add(new HumanPlayer(humanPlayerStrategyChooser.get(), HumanPlayerSource.BLUE_LEFT));
            humanPlayers.add(new HumanPlayer(humanPlayerStrategyChooser.get(), HumanPlayerSource.BLUE_RIGHT));
        }
    }

    public static class HumanPlayer {

        public static final int MAX_CORAL_PER_STATION = 30;
        public static final double GROUND_NEAR_VELOCITY = 1.0;
        public static final double GROUND_FAR_VELOCITY = 4.0;
        public static final double SOURCE_VELOCITY = 2.0;

        private static final double SOURCE_WIDTH = 0.75;
        private static final double INTAKE_OFFSET = 0.1;
        private static final double SOURCE_HEIGHT = 1.16;
        private static final double CORAL_GROUND_TOUCH_HEIGHT = 0.2;

        private static final double GROUND_INTERVAL_MIN = 1.0;
        private static final double GROUND_INTERVAL_MAX = 2.3;
        private static final double SOURCE_INTERVAL_MIN = 1.4;
        private static final double SOURCE_INTERVAL_MAX = 2.0;

        private double lastDropTime = 0;
        private int coralLeft = MAX_CORAL_PER_STATION;
        private Random random = new Random();
        private HumanPlayerSimulationStrategy strategy;
        private HumanPlayerSource source;

        public HumanPlayer(HumanPlayerSimulationStrategy strategy, HumanPlayerSource source) {
            this.strategy = strategy;
            this.source = source;
        }

        public void setStrategy(HumanPlayerSimulationStrategy strategy) {
            this.strategy = strategy;
        }

        public HumanPlayerSimulationStrategy getStrategy() {
            return strategy;
        }

        public void setSource(HumanPlayerSource source) {
            this.source = source;
        }

        public HumanPlayerSource getSource() {
            return source;
        }

        public void reset() {
            lastDropTime = 0;
            coralLeft = MAX_CORAL_PER_STATION;
        }

        private Pose2d getSourcePose() {
            return switch (source) {
                case RED_LEFT -> Constants.Game.SourceCoralSpawnPosition.RED_CORAL_LEFT.getPose();
                case RED_RIGHT -> Constants.Game.SourceCoralSpawnPosition.RED_CORAL_RIGHT.getPose();
                case BLUE_LEFT -> Constants.Game.SourceCoralSpawnPosition.BLUE_CORAL_LEFT.getPose();
                case BLUE_RIGHT -> Constants.Game.SourceCoralSpawnPosition.BLUE_CORAL_RIGHT.getPose();
            };
        }

        private Pose3d getEjectPose(Pose2d target) {
            Pose3d sourcePose = new Pose3d(getSourcePose());

            Pose3d relativeTarget = new Pose3d(target).relativeTo(sourcePose);

            return sourcePose.transformBy(new Transform3d(
                    MathUtil.clamp(relativeTarget.getX() + INTAKE_OFFSET, -SOURCE_WIDTH, SOURCE_WIDTH),
                    0,
                    SOURCE_HEIGHT,
                    new Rotation3d(0, 0, 0)));
        }

        public void dropWithVelocity(Pose2d target, double velocity) {
            if (coralLeft <= 0) {
                return;
            }

            coralLeft--;

            Pose3d ejectPose = getEjectPose(target);
            Translation3d xyzVelocity = new Translation3d(0, velocity, 0).rotateBy(ejectPose.getRotation());

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new GamePieceProjectile(
                                    ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                                    ejectPose.toPose2d().getTranslation(),
                                    xyzVelocity.toTranslation2d(),
                                    ejectPose.getZ(),
                                    xyzVelocity.getZ(),
                                    ejectPose.getRotation())
                            .withTouchGroundHeight(CORAL_GROUND_TOUCH_HEIGHT)
                            .enableBecomesGamePieceOnFieldAfterTouchGround());
        }

        private void runOnlyGroundNear() {
            if (Timer.getTimestamp() - lastDropTime > random.nextDouble(GROUND_INTERVAL_MIN, GROUND_INTERVAL_MAX)) {
                lastDropTime = Timer.getTimestamp();
                dropWithVelocity(RobotContainer.model.getRobot(), GROUND_NEAR_VELOCITY);
            }
        }

        private void runOnlyGroundFar() {
            if (Timer.getTimestamp() - lastDropTime > random.nextDouble(GROUND_INTERVAL_MIN, GROUND_INTERVAL_MAX)) {
                lastDropTime = Timer.getTimestamp();
                dropWithVelocity(RobotContainer.model.getRobot(), GROUND_FAR_VELOCITY);
            }
        }

        private void runOnlySource() {
            if (Timer.getTimestamp() - lastDropTime > random.nextDouble(SOURCE_INTERVAL_MIN, SOURCE_INTERVAL_MAX)
                    && RobotContainer.model
                                    .getRobot()
                                    .getTranslation()
                                    .getDistance(getSourcePose().getTranslation())
                            < 1.0) {
                lastDropTime = Timer.getTimestamp();
                dropWithVelocity(RobotContainer.model.getRobot(), SOURCE_VELOCITY);
            }
        }

        private void runGroundWhenOccupied() {
            if (Timer.getTimestamp() - lastDropTime > random.nextDouble(SOURCE_INTERVAL_MIN, SOURCE_INTERVAL_MAX)) {
                if (RobotContainer.model
                                .getRobot()
                                .getTranslation()
                                .getDistance(getSourcePose().getTranslation())
                        < 1.0) {
                    lastDropTime = Timer.getTimestamp();
                    dropWithVelocity(RobotContainer.model.getRobot(), SOURCE_VELOCITY);
                } else if (
                /*TODO: is occupied */ false) {
                    lastDropTime = Timer.getTimestamp();
                    dropWithVelocity(RobotContainer.model.getRobot(), GROUND_FAR_VELOCITY);
                }
            }
        }

        public void periodic() {
            switch (strategy) {
                case NONE -> {
                    /* disable human player sim */
                }
                case ONLY_GROUND_NEAR -> runOnlyGroundNear();
                case ONLY_GROUND_FAR -> runOnlyGroundFar();
                case ONLY_SOURCE -> runOnlySource();
                case GROUND_WHEN_OCCUPIED -> runGroundWhenOccupied();
            }
        }
    }

    public void reset() {
        for (HumanPlayer player : humanPlayers) {
            player.reset();
        }
    }

    @Override
    public void simulationPeriodic() {
        for (HumanPlayer player : humanPlayers) {
            player.setStrategy(humanPlayerStrategyChooser.get());
            if (DriverStation.isEnabled()) {
                player.periodic();
            }
        }
    }
}
