// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax leftMotor; 
  private CANSparkMax rightMotor;

  private CANSparkMax pivotMotor;


  private DutyCycleEncoder shooterEncoder;
 
  //private CANSparkMax pivotMotor; not implemented yet

  /** Creates a new Shooter. */
  public Shooter() {
    leftMotor = new CANSparkMax(Constants.ShooterConstants.leftCANSparkID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.ShooterConstants.rightCANSparkID, MotorType.kBrushless);

    pivotMotor = new CANSparkMax(Constants.ShooterConstants.pivotCANSparkID, MotorType.kBrushless);

    shooterEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterEncoderID);
    shooterEncoder.setPositionOffset(Constants.ShooterConstants.shooterEncoderOffset);
    

    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
    pivotMotor.setIdleMode(IdleMode.kBrake);


  }


  public void setSpeed(double speed){
    //ASSUMING RIGHT MOTOR NEEDS TO SPIN IN NEGATIVE DIRECTION
    leftMotor.set(speed); //was negative
    rightMotor.set(speed);
  }
  public void stopShooter() {
    //For Autonomous
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }

  
  public void zeroEncoder(){
    shooterEncoder.reset();
  }

  public double getLeftMotorVoltage(){
    return leftMotor.getBusVoltage();
  }

  public double getRightMotorVoltage(){
    return rightMotor.getBusVoltage();
  }

  public double getPivotMotorVoltage(){
    return pivotMotor.getBusVoltage();
  }

  public RelativeEncoder getLeftEncoder() {
    return leftMotor.getEncoder();
  }

  public RelativeEncoder getRightEncoder() {
    return rightMotor.getEncoder();
  }
  public DutyCycleEncoder getShooterEncoder(){
    return shooterEncoder;
  }
  public double getAbsoluteDistance(){
    return Math.abs(shooterEncoder.getDistance());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder Position", getShooterEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Shooter encoder distance: ", Math.abs(getShooterEncoder().getDistance()));
    // This method will be called once per scheduler run
  }
}
