package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.lights.StateVisualizerLights;
import frc.robot.utils.ManagedSubsystemBase;

public class Lights extends ManagedSubsystemBase {

    public final StateVisualizerLights stateVisualizer;

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    public Lights() {
        leds = new AddressableLED(RobotMap.Lights.LED_ID);
        buffer = new AddressableLEDBuffer(Constants.Lights.LENGTH);
        leds.setColorOrder(ColorOrder.kRGB);
        leds.setLength(Constants.Lights.LENGTH);
        leds.start();

        stateVisualizer = new StateVisualizerLights(this);
    }

    @Override
    public void periodicManaged() {
        // Send the latest LED color data to the LED strip
        leds.setData(buffer);
    }

    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }

    @Override
    public void close() {
        leds.close();
    }
}
