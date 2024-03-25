// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.proto.System;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class lightingSubsystem extends SubsystemBase {
  /** Creates a new lightingSubsystem. */
    private static AddressableLED led_strip;
    private static AddressableLEDBuffer led_buffer;
    private int red = 0;
    private int green = 0;
    private int blue = 0;
    private int time = 0;
    private boolean blink = false;
    private boolean blinkOn = false;
    private double currentTime;
    
    


  public lightingSubsystem() {
    led_strip = new AddressableLED(0);

    led_buffer = new AddressableLEDBuffer(30);

    led_strip.setLength(led_buffer.getLength());

    rgBlink(255, 0, 0, 1);
  }

  public void setRed(int value){
    if(value < 0) value = 0;
    else if (value > 255) value = 255;
    // redPWM.set(value);

    for (int i = 0; i < led_buffer.getLength(); i++) {
      led_buffer.setRGB(i, value, 0, 0);
    }

    
    led_strip.setData(led_buffer);
    led_strip.start();
  }

  public void setGreen(int value){
    if(value < 0) value = 0;
    else if (value > 255) value = 255;

    for (int pixel = 0; pixel < led_buffer.getLength(); pixel++) {
      led_buffer.setRGB(pixel, 0, value, 0);
    }

    led_strip.setData(led_buffer);
    led_strip.start();
  }
  public boolean setBlue(int value){
    if(value < 0) value = 0;
    else if (value > 255) value = 255;

    for (int pixel = 0; pixel < led_buffer.getLength(); pixel++) {
      led_buffer.setRGB(pixel, 0, 0, value);
    }

    led_strip.setData(led_buffer);
    led_strip.start();
    return true;
  }
  public void setRGB(int red, int green, int blue){
    if(red < 0) red = 0;
    else if (red > 255) red = 255;
    if(green < 0) green = 0;
    else if (green > 255) green = 255;
    if(blue < 0) blue = 0;
    else if (blue > 255) blue = 255;


    for (int pixel = 0; pixel < led_buffer.getLength(); pixel++) {
      led_buffer.setRGB(pixel, red, green, blue);
    }

    led_strip.setData(led_buffer);
    led_strip.start();
  }
  public void rgBlink(int red, int green, int blue, int time){
    this.red = red;
    this.green = green;
    this.blue = blue;
    this.time = time;
    blink = true;

    setRGB(red, green, blue);

    blinkOn = true;

    currentTime = Timer.getFPGATimestamp();

  }

  // public static void setBlue(double setPoint){
  //   if(setPoint < 0) setPoint = 0;
  //   else if (setPoint > 1) setPoint = 1;
  //   bluePWM.set(setPoint);
  // }

  @Override
  public void periodic() {
      if (blink){
        if(blinkOn && currentTime > 1){
          setRGB(0, 0, 0);
          blinkOn = false;
          currentTime = Timer.getFPGATimestamp();
        }else if (!blinkOn && currentTime > 1) {
          setRGB(red, green, blue);
          blinkOn = true;
          currentTime = Timer.getFPGATimestamp();
        }
      }
  }
}