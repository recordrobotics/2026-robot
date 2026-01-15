package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID;

@SuppressWarnings({ // This class is based off WPILib's Joystick class, so suppress warnings
    "java:S115",
    "java:S2047"
})
public class MacroPad extends GenericHID {

    public enum Button {
        kJButton(1),
        kN1(2),
        kN2(3),
        kN3(4),
        kNA(5),
        kN4(6),
        kN5(7),
        kN6(8),
        kNB(9),
        kN7(10),
        kN8(11),
        kN9(12),
        kNC(13),
        kNStar(14),
        kN0(15),
        kNHash(16),
        kND(17);

        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and appending `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            // Remove leading `k`
            return this.name().substring(1) + "Button";
        }
    }

    public enum Axis {
        /** Left X axis. */
        kLeftX(0),
        /** Left Y axis. */
        kLeftY(1);

        /** Axis value. */
        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This is done by
         * stripping the leading `k`, and appending `Axis` if the name ends with `Trigger`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
            return this.name().substring(1);
        }
    }

    public MacroPad(int port) {
        super(port);
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return getRawAxis(Axis.kLeftX.value);
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return getRawAxis(Axis.kLeftY.value);
    }

    public boolean getButton(Button button) {
        return getRawButton(button.value);
    }

    public boolean getButtonPressed(Button button) {
        return getRawButtonPressed(button.value);
    }

    public boolean getButtonReleased(Button button) {
        return getRawButtonReleased(button.value);
    }

    public boolean getJButton() {
        return getRawButton(Button.kJButton.value);
    }

    public boolean getJButtonPressed() {
        return getRawButtonPressed(Button.kJButton.value);
    }

    public boolean getJButtonReleased() {
        return getRawButtonReleased(Button.kJButton.value);
    }

    public boolean getN1() {
        return getRawButton(Button.kN1.value);
    }

    public boolean getN1Pressed() {
        return getRawButtonPressed(Button.kN1.value);
    }

    public boolean getN1Released() {
        return getRawButtonReleased(Button.kN1.value);
    }

    public boolean getN2() {
        return getRawButton(Button.kN2.value);
    }

    public boolean getN2Pressed() {
        return getRawButtonPressed(Button.kN2.value);
    }

    public boolean getN2Released() {
        return getRawButtonReleased(Button.kN2.value);
    }

    public boolean getN3() {
        return getRawButton(Button.kN3.value);
    }

    public boolean getN3Pressed() {
        return getRawButtonPressed(Button.kN3.value);
    }

    public boolean getN3Released() {
        return getRawButtonReleased(Button.kN3.value);
    }

    public boolean getNA() {
        return getRawButton(Button.kNA.value);
    }

    public boolean getNAPressed() {
        return getRawButtonPressed(Button.kNA.value);
    }

    public boolean getNAReleased() {
        return getRawButtonReleased(Button.kNA.value);
    }

    public boolean getN4() {
        return getRawButton(Button.kN4.value);
    }

    public boolean getN4Pressed() {
        return getRawButtonPressed(Button.kN4.value);
    }

    public boolean getN4Released() {
        return getRawButtonReleased(Button.kN4.value);
    }

    public boolean getN5() {
        return getRawButton(Button.kN5.value);
    }

    public boolean getN5Pressed() {
        return getRawButtonPressed(Button.kN5.value);
    }

    public boolean getN5Released() {
        return getRawButtonReleased(Button.kN5.value);
    }

    public boolean getN6() {
        return getRawButton(Button.kN6.value);
    }

    public boolean getN6Pressed() {
        return getRawButtonPressed(Button.kN6.value);
    }

    public boolean getN6Released() {
        return getRawButtonReleased(Button.kN6.value);
    }

    public boolean getNB() {
        return getRawButton(Button.kNB.value);
    }

    public boolean getNBPressed() {
        return getRawButtonPressed(Button.kNB.value);
    }

    public boolean getNBReleased() {
        return getRawButtonReleased(Button.kNB.value);
    }

    public boolean getN7() {
        return getRawButton(Button.kN7.value);
    }

    public boolean getN7Pressed() {
        return getRawButtonPressed(Button.kN7.value);
    }

    public boolean getN7Released() {
        return getRawButtonReleased(Button.kN7.value);
    }

    public boolean getN8() {
        return getRawButton(Button.kN8.value);
    }

    public boolean getN8Pressed() {
        return getRawButtonPressed(Button.kN8.value);
    }

    public boolean getN8Released() {
        return getRawButtonReleased(Button.kN8.value);
    }

    public boolean getN9() {
        return getRawButton(Button.kN9.value);
    }

    public boolean getN9Pressed() {
        return getRawButtonPressed(Button.kN9.value);
    }

    public boolean getN9Released() {
        return getRawButtonReleased(Button.kN9.value);
    }

    public boolean getNC() {
        return getRawButton(Button.kNC.value);
    }

    public boolean getNCPressed() {
        return getRawButtonPressed(Button.kNC.value);
    }

    public boolean getNCReleased() {
        return getRawButtonReleased(Button.kNC.value);
    }

    public boolean getNStar() {
        return getRawButton(Button.kNStar.value);
    }

    public boolean getNStarPressed() {
        return getRawButtonPressed(Button.kNStar.value);
    }

    public boolean getNStarReleased() {
        return getRawButtonReleased(Button.kNStar.value);
    }

    public boolean getN0() {
        return getRawButton(Button.kN0.value);
    }

    public boolean getN0Pressed() {
        return getRawButtonPressed(Button.kN0.value);
    }

    public boolean getN0Released() {
        return getRawButtonReleased(Button.kN0.value);
    }

    public boolean getNHash() {
        return getRawButton(Button.kNHash.value);
    }

    public boolean getNHashPressed() {
        return getRawButtonPressed(Button.kNHash.value);
    }

    public boolean getNHashReleased() {
        return getRawButtonReleased(Button.kNHash.value);
    }

    public boolean getND() {
        return getRawButton(Button.kND.value);
    }

    public boolean getNDPressed() {
        return getRawButtonPressed(Button.kND.value);
    }

    public boolean getNDReleased() {
        return getRawButtonReleased(Button.kND.value);
    }
}
