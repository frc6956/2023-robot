// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  public static int[] green = {0, 255, 0};
  public static int[] greenNew = {0, 255, 0};
  public static int[] red = {255, 0, 0};
  public static int[] blue = {0, 0, 255};
  public static int[] gold = {50, 20, 0};
  public static int[] pink = {255, 0, 70};
  public static int[] purple = {180, 0, 255};
  public static int[] yellow = {255, 140, 0};


  double m_rainbowFirstPixelHue = 0;
  double m_rainbowFirstPixelHue2 = 180;

  /** Creates a new LEDs. */
  public LEDs() {
    //PWM port 1 on the Rio
    m_led = new AddressableLED(Constants.PWM.leds);


    //sets the length of the LEDs
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDs.length);


    setUpLight();
  }

  public void setUpLight() {
    //defines the length of the leds
    m_led.setLength(m_ledBuffer.getLength());

    //sets the LED data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setAllRGBColor(int[] color){

    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      //Sets each LED to the same color

      if (i %2 == 0){
        m_ledBuffer.setRGB(i, (int)(color[0]*.1), (int)(color[1]*.1), (int)(color[2]*.1));
        //m_ledBuffer.setRGB(i, 0, 0, 0);
      } else {
        m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
      }
      
    }
    //updates every LED to their set color
    m_led.setData(m_ledBuffer);
  }

  public void setAllOff(){ // sets every LED to a color of 0 or off
    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }

    m_led.setData(m_ledBuffer);
  }


  public void rainbow(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      final int hue = ((int)(m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180;

      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    // makes the rainbow move
    m_rainbowFirstPixelHue += 1;

    m_rainbowFirstPixelHue %= 180;
  }

  public void rainbowReverse(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      final int hue = ((int)(m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180;
      final int hue2 = ((int)(m_rainbowFirstPixelHue2) - (i * 180 / m_ledBuffer.getLength())) % 180;

      if (i > m_ledBuffer.getLength()/2){
        m_ledBuffer.setHSV((m_ledBuffer.getLength() - i), hue2, 255, 64);
      } else {
        m_ledBuffer.setHSV(i, hue, 255, 64);
      }
    }
    // makes the rainbow move
    m_rainbowFirstPixelHue += 1;
    m_rainbowFirstPixelHue2 -=1;

    m_rainbowFirstPixelHue2 %= 180;
    m_rainbowFirstPixelHue %= 180;
  }


  public void dimRainbow(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++){
      final int hue = ((int)(m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180;
      if (i %2 == 0){
        
        m_ledBuffer.setHSV(i, hue, 255, 40);
      } else {
        m_ledBuffer.setHSV(i, hue, 255, 255);
      }

      m_rainbowFirstPixelHue += 0.05;

      m_rainbowFirstPixelHue %= 180;

    }

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
  }
}
