// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax rollerMotor;
  private CANSparkMax armMotor;
  private DigitalInput intakeSensor;
  //private Encoder armEncoder;
  private DutyCycleEncoder armEncoder;



  /** Creates a new Intake. */
  public Intake() {
    


    rollerMotor = new CANSparkMax(Constants.IntakeConstants.rollerCANSparkID, MotorType.kBrushless);
    armMotor = new CANSparkMax(Constants.IntakeConstants.pivotCANSparkID, MotorType.kBrushless);

    intakeSensor = new DigitalInput(Constants.IntakeConstants.IntakeSensorID);
    


    //armEncoder = new Encoder(0, 2, 3);
    armEncoder = new DutyCycleEncoder(1);
    armEncoder.setPositionOffset(0.65);
    

    rollerMotor.setIdleMode(IdleMode.kCoast);
    armMotor.setIdleMode(IdleMode.kCoast);

    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);


  }
  
  public void setRollerSpeed(double rollerSpeed) {
    rollerMotor.set(rollerSpeed);
  }

  public void stopRollerSpeed() {
    rollerMotor.set(0);
  }

  public double getRollerSpeed() {
    return rollerMotor.get();
  }

  public void setArmSpeed(double armSpeed){
    armMotor.setSmartCurrentLimit(50); //check

    
    armMotor.set(armSpeed);
  }

  public double getArmVoltage(){
    return armMotor.getBusVoltage();
  }

  public double getArmSpeed() {
    return armMotor.get();
  }

  public RelativeEncoder getRollerEncoder() {
    return rollerMotor.getEncoder();
  }

  public DutyCycleEncoder getArmEncoder() {
    //return armEncoder;
    return armEncoder;
  }

  public DigitalInput getDigitalInput() {
    return intakeSensor;
  }


  public void SetCoast() {
    rollerMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrake() {
    rollerMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putBoolean("Arm Encoder Online: ", getArmEncoder().isConnected());
     SmartDashboard.putNumber("Arm Encoder Distance: ", getArmEncoder().getDistance());
     SmartDashboard.putNumber("Arm Encoder: ", getArmEncoder().getAbsolutePosition());
    SmartDashboard.putBoolean("Arm Sensor Value", intakeSensor.get());

  
  }

}
