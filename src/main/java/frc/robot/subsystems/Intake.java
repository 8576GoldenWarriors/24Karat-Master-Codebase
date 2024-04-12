// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //private CANSparkMax rollerMotor;
  private CANSparkMax armMotor;
 
  private Encoder armEncoder;
  //private DutyCycleEncoder armEncoder;
  //private Encoder armEncoder



  /** Creates a new Intake. */
  public Intake() {
    
    armMotor = new CANSparkMax(Constants.IntakeConstants.pivotCANSparkID, MotorType.kBrushless);
    // armEncoder = new DutyCycleEncoder(Constants.IntakeConstants.intakeEncoderPort);
    // armEncoder.setPositionOffset(0.75);

    armEncoder = new Encoder(Constants.IntakeConstants.intakeEncoderA, 
                            Constants.IntakeConstants.intakeEncoderB, Constants.IntakeConstants.intakeEncoderI);
    

    //rollerMotor.setIdleMode(IdleMode.kCoast);
    armMotor.setIdleMode(IdleMode.kCoast);

    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);


  }
  
  

  public void setArmSpeed(double armSpeed){
    armMotor.setSmartCurrentLimit(55); //check

    
    armMotor.set(armSpeed);
  }

  public double getArmVoltage(){
    return armMotor.getBusVoltage();
  }

  public double getArmSpeed() {
    return armMotor.get();
  }

  // public RelativeEncoder getRollerEncoder() {
  //   return rollerMotor.getEncoder();
  // }

  // public DutyCycleEncoder getArmEncoder() {
  //   //return armEncoder;
  //   return armEncoder;
  // }
  public Encoder getArmEncoder() {
    //return armEncoder;
    return armEncoder;
  }


  public void zeroEncoder(){
    armEncoder.reset();
  }

  

  @Override
  public void periodic() {
    
    //SmartDashboard.putBoolean("Arm Encoder Online: ", getArmEncoder().isConnected());
     SmartDashboard.putNumber("Arm Encoder Distance: ", getArmEncoder().getDistance());
     //SmartDashboard.putNumber("Arm Encoder: ", getArmEncoder().getAbsolutePosition());
    

  
  }

}
