// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterRoller extends SubsystemBase {
  /** Creates a new ShooterRoller. */
  private CANSparkMax leftMotor; 
  private CANSparkMax rightMotor;

  private boolean revved;
  private boolean amping;

  public ShooterRoller() {
    leftMotor = new CANSparkMax(Constants.ShooterConstants.leftCANSparkID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.ShooterConstants.rightCANSparkID, MotorType.kBrushless);

    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);

    revved = false;
    amping = false;



  }

  public void setSpeed(double speed){
    // revved = true;
    // if(amping){
    //   amping = false;
    // }
    //ASSUMING RIGHT MOTOR NEEDS TO SPIN IN NEGATIVE DIRECTION
   leftMotor.set(speed); //was negative
    rightMotor.set(speed);
  }
  public void setLeft(double speed){
    //ASSUMING RIGHT MOTOR NEEDS TO SPIN IN NEGATIVE DIRECTION
   leftMotor.set(speed); //was negative
    
  }
  public void setRight(double speed){
    //ASSUMING RIGHT MOTOR NEEDS TO SPIN IN NEGATIVE DIRECTION
   
    rightMotor.set(speed);
  }
  public void stopShooter() {
    //For Autonomous
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public double getLeftMotorVoltage(){
    return leftMotor.getBusVoltage();
  }

  public double getRightMotorVoltage(){
    return rightMotor.getBusVoltage();
  }
   public RelativeEncoder getLeftEncoder() {
    return leftMotor.getEncoder();
  }

  public RelativeEncoder getRightEncoder() {
    return rightMotor.getEncoder();
  }



  public boolean isRevved(){
    return revved;
  }

  public boolean isAmping(){
    return amping;
  }

  public void setRevved(boolean isRevved){
    revved = isRevved;
    if(amping){
      amping = false;
    }
  }

  public void setAmping(boolean isAmping){
    amping = isAmping;
    if(revved){
      revved = false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Shooter Module Current: ", getRightMotorVoltage());
    SmartDashboard.putNumber("Bottom Shooter Module Current: ", getLeftMotorVoltage());
    

  }
}
