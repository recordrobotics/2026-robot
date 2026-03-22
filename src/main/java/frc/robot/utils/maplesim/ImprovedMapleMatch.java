package frc.robot.utils.maplesim;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Random;

public class ImprovedMapleMatch {

    private static final Random RANDOM = new Random();
    private static final String RED_ACTIVE_KEY = "MapleSim/MatchData/Breakdown/Red Alliance/Improved Active";
    private static final String BLUE_ACTIVE_KEY = "MapleSim/MatchData/Breakdown/Blue Alliance/Improved Active";

    private static ImprovedMapleMatch instance;

    public static ImprovedMapleMatch getInstance() {
        if (instance == null) {
            instance = new ImprovedMapleMatch();
        }
        return instance;
    }

    private double oldRedFuel = 0;
    private double oldBlueFuel = 0;
    private double redFuelWhileActive = 0;
    private double blueFuelWhileActive = 0;
    private boolean hasTeleopStarted = false;
    private boolean redInactiveFirst = false;

    private boolean redActive = false;
    private boolean blueActive = false;

    private double shiftEndTime = 0;
    private boolean nextShiftRedActive = false;
    private boolean nextShiftBlueActive = false;

    public ImprovedMapleMatch() {
        sendActiveState();
        sendScore();
    }

    public void sendActiveState() {
        double timeLeftInShift = shiftEndTime - Timer.getTimestamp();
        boolean flash = hasTeleopStarted
                && timeLeftInShift < 3
                && (timeLeftInShift % 0.5 < 0.25); // Flash in the last 3 seconds of a shift

        SmartDashboard.putBoolean(RED_ACTIVE_KEY, redActive && (!flash || nextShiftRedActive));
        SmartDashboard.putBoolean(BLUE_ACTIVE_KEY, blueActive && (!flash || nextShiftBlueActive));
    }

    public void periodic() {
        updateActiveStates();

        double redFuel = SmartDashboard.getNumber("MapleSim/MatchData/Breakdown/Red Alliance/TotalFuelInHub", 0);
        double blueFuel = SmartDashboard.getNumber(
                "MapleSim/MatchData/Breakdown/blue Alliance/TotalFuelInHub", 0); // maplesim typo

        if (redFuel > oldRedFuel) {
            if (redActive) {
                redFuelWhileActive += redFuel - oldRedFuel;
            }
            oldRedFuel = redFuel;
        } else if (redFuel < oldRedFuel) {
            oldRedFuel = redFuel;
        }

        if (blueFuel > oldBlueFuel) {
            if (blueActive) {
                blueFuelWhileActive += blueFuel - oldBlueFuel;
            }
            oldBlueFuel = blueFuel;
        } else if (blueFuel < oldBlueFuel) {
            oldBlueFuel = blueFuel;
        }

        sendActiveState();
        sendScore();
    }

    public void autonomousInit() {
        redFuelWhileActive = 0;
        blueFuelWhileActive = 0;
        hasTeleopStarted = false;
        nextShiftRedActive = false;
        nextShiftBlueActive = false;

        DriverStationSim.setGameSpecificMessage("");
        redActive = true;
        blueActive = true;
        sendActiveState();
        sendScore();
    }

    public void teleopInit() {
        hasTeleopStarted = true;

        if (oldRedFuel > oldBlueFuel) {
            redInactiveFirst = true;
        } else if (oldBlueFuel > oldRedFuel) {
            redInactiveFirst = false;
        } else {
            redInactiveFirst = RANDOM.nextBoolean();
        }

        DriverStationSim.setGameSpecificMessage(redInactiveFirst ? "R" : "B");
    }

    public void updateActiveStates() {
        if (!hasTeleopStarted) {
            redActive = true;
            blueActive = true;
        } else if (DriverStation.isTeleopEnabled()) {
            double matchTime = Math.floor(DriverStation.getMatchTime());

            if (matchTime > 130) {
                // Transition shift, hub is active.
                redActive = true;
                blueActive = true;
                if (matchTime >= 130 + 3) {
                    nextShiftRedActive = !redInactiveFirst;
                    nextShiftBlueActive = redInactiveFirst;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else if (matchTime > 105) {
                // Shift 1
                redActive = !redInactiveFirst;
                blueActive = redInactiveFirst;
                if (matchTime >= 105 + 3) {
                    nextShiftRedActive = redInactiveFirst;
                    nextShiftBlueActive = !redInactiveFirst;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else if (matchTime > 80) {
                // Shift 2
                redActive = redInactiveFirst;
                blueActive = !redInactiveFirst;
                if (matchTime >= 80 + 3) {
                    nextShiftRedActive = !redInactiveFirst;
                    nextShiftBlueActive = redInactiveFirst;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else if (matchTime > 55) {
                // Shift 3
                redActive = !redInactiveFirst;
                blueActive = redInactiveFirst;
                if (matchTime >= 55 + 3) {
                    nextShiftRedActive = redInactiveFirst;
                    nextShiftBlueActive = !redInactiveFirst;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else if (matchTime > 30) {
                // Shift 4
                redActive = redInactiveFirst;
                blueActive = !redInactiveFirst;
                if (matchTime >= 30 + 3) {
                    nextShiftRedActive = true;
                    nextShiftBlueActive = true;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else {
                // End game, hub always active.
                redActive = true;
                blueActive = true;
                if (matchTime >= 3) {
                    nextShiftRedActive = false;
                    nextShiftBlueActive = false;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            }
        } else {
            redActive = false;
            blueActive = false;
        }
    }

    public void sendScore() {
        SmartDashboard.putNumber("MapleSim/MatchData/Breakdown/Red Alliance/Improved Score", redFuelWhileActive);
        SmartDashboard.putNumber("MapleSim/MatchData/Breakdown/Blue Alliance/Improved Score", blueFuelWhileActive);
    }
}
