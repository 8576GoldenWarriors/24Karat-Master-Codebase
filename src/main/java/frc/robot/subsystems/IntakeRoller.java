// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase {
  private CANSparkMax rollerMotor;
   private DigitalInput intakeSensor;
  /** Creates a new IntakeRoller. */
  public IntakeRoller() {
    intakeSensor = new DigitalInput(Constants.IntakeConstants.IntakeSensorID);
    rollerMotor = new CANSparkMax(Constants.IntakeConstants.rollerCANSparkID, MotorType.kBrushless);
    rollerMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setRollerSpeed(double speed){
    rollerMotor.set(speed);
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note Sensor Value", intakeSensor.get());
   
    // This method will be called once per scheduler run
  }

  public void stopRollerSpeed() {
    rollerMotor.set(0);
  }

  public double getRollerSpeed() {
    return rollerMotor.get();
  }


  public void SetCoast() {
    rollerMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrake() {
    rollerMotor.setIdleMode(IdleMode.kBrake);
  }

  public DigitalInput getDigitalInput(){
    return intakeSensor;
  }
}
