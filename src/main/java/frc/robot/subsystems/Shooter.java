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
  

  private CANSparkMax pivotMotor;


  private DutyCycleEncoder shooterEncoder;
 
  //private CANSparkMax pivotMotor; not implemented yet

  /** Creates a new Shooter. */
  public Shooter() {
    

    pivotMotor = new CANSparkMax(Constants.ShooterConstants.pivotCANSparkID, MotorType.kBrushless);

    shooterEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterEncoderID);
    shooterEncoder.setPositionOffset(Constants.ShooterConstants.shooterEncoderOffset);
    

    
    pivotMotor.setIdleMode(IdleMode.kBrake);


  }

  public CANSparkMax getShooterMotor(){
    return pivotMotor;
  }


  

  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }

  
  public void zeroEncoder(){
    shooterEncoder.reset();
  }

  

  public double getPivotMotorVoltage(){
    return pivotMotor.getBusVoltage();
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
