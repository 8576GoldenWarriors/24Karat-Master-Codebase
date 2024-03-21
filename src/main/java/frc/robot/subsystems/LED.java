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
  int colorID;

  public LED(int port, int length) {
//     led = new AddressableLED(port);
//     ledBuffer = new AddressableLEDBuffer(length);
//     this.length = length;
//     led.setLength(length);
    

//     for(int i=0; i<length; i++){
//       ledBuffer.setHSV(i, 0, 100, 100);
//     }

//     led.setData(ledBuffer);

//     led.start();

//     colorID = 0;

    
  }
//   public void setLED(int colorID){
//     if(colorID == 1  && !(this.colorID==1)){
      
//       green();
//     }
//     else{
//       gold();
//     }

//     if(colorID == 0  && !(this.colorID ==0)){
//       gold();
//     }
//     else{
//       green();
//     }
//   }

//   public void green(){
//     for(int i=0; i<length; i++){
//       ledBuffer.setHSV(i, 126, 69, 100);
//     }
//     colorID = 1;
//     led.setData(ledBuffer);
//    // System.out.println("green");
//   }

//   public void gold(){
//     for(int i=0; i<length; i++){
//       ledBuffer.setHSV(i, 45, 100, 100);
//     }
//     colorID = 0;
//     led.setData(ledBuffer);
//     //System.out.println("gold");
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run

//     led.setData(ledBuffer);

//     led.start();
//   }
}
