// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax rollerMotor;
  private CANSparkMax armMotor;
  public DigitalInput photoelectric;

  /** Creates a new Intake. */
  public Intake() {
    photoelectric = new DigitalInput(Constants.IntakeConstants.IntakeSensorID);
    rollerMotor = new CANSparkMax(Constants.IntakeConstants.rollerCANSparkID, MotorType.kBrushless);
    armMotor = new CANSparkMax(Constants.IntakeConstants.pivotCANSparkID, MotorType.kBrushless);

    rollerMotor.setIdleMode(IdleMode.kCoast);
    armMotor.setIdleMode(IdleMode.kCoast);


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
    armMotor.setSmartCurrentLimit(5);

    
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

  public RelativeEncoder getArmEncoder() {
    return armMotor.getEncoder();
  }

  public DigitalInput getDigitalInput() {
    return photoelectric;
  }


  public void SetCoast() {
    rollerMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrake() {
    rollerMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
     SmartDashboard.putNumber("Arm Encoder: ", getArmEncoder().getPosition());
     SmartDashboard.putNumber("Num ticks arm encoder: ", getArmEncoder().getCountsPerRevolution());
     SmartDashboard.putNumber("Conversion factor", getArmEncoder().getPositionConversionFactor());
    // This method will be called once per scheduler run
  }

}
