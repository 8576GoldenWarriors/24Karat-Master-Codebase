// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  int length;
  String color;

  public LED(int port, int length) {
    //led = new AddressableLED(port);
    // ledBuffer = new AddressableLEDBuffer(length);
    // this.length = length;
    // led.setLength(length);

    // for(int i=0; i<length; i++){
    //   ledBuffer.setHSV(i, 0, 100, 100);
    // }

    // led.setData(ledBuffer);

    // led.start();

    // color = "red";
  }
  public void setLED(String color){
    if(color.equals("green") && !(this.color.equals("green"))){
      green();
    }
    if(color.equals("red") && !(this.color.equals("red"))){
      red();
    }
  }

  public void green(){
    for(int i=0; i<length; i++){
      ledBuffer.setHSV(i, 126, 69, 100);
    }
    color = "green";
    led.setData(ledBuffer);
  }
  public void red(){
    for(int i=0; i<length; i++){
      ledBuffer.setHSV(i, 0, 100, 100);
    }
    color = "red";
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
