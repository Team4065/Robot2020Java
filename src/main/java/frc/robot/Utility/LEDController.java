// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Add your docs here.
 */
public class LEDController {

    private AddressableLED  ledController;
    private AddressableLEDBuffer ledBuffer;

    private short rainbowFirstPixelHue = 0;

    public LEDController(int PWMPort, int numberOfLEDs){
        ledController = new AddressableLED(PWMPort);
        ledController.setLength(numberOfLEDs);
        ledBuffer = new AddressableLEDBuffer(numberOfLEDs);

        ledController.setData(ledBuffer);
        ledController.start();
    }

    /**
     * Sets the entire LED strip to one color
     * @param red 0-255
     * @param green 0-255
     * @param blue 0-255
     */
    public void setMonoColor(short red, short green, short blue){
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, red, green, blue);
        }
        
        ledController.setData(ledBuffer);
    }

    /**
     * Sets the value of a single led
     * @param ledIndex
     * @param red 0-255
     * @param green 0-255
     * @param blue 0-255
     */
    public void setIndividualLED(int ledIndex, short red, short green, short blue){
        ledBuffer.setRGB(ledIndex, red, green, blue);
        ledController.setData(ledBuffer);
    }

    /**
     * Progresses the rainbow animation
     */
    public void updateRainbow(){
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }
}
