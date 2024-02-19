// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeUp extends Command {
  /** Creates a new IntakeUp. */
  public Intake intake;

  double setpoint;
  double kP;
  double Ki;
  double Kd;
  double error;
  double lastError;
  double integral;
  double derivative;
  double avgPos;
  double motorPower;

  Boolean done = false;

  public IntakeUp(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // setpoint = Constants.IntakeConstants.kUpPosition;
    // kP = 0.07;
    // Ki = 0;
    // Kd = 0.01;
    // lastError = 0;
    // integral = 0;
    // derivative = 0;
    // avgPos = intake.getArmEncoder().getPosition();
    // motorPower = 0;

    // error = setpoint - avgPos;
    // integral = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setArmSpeed(Constants.IntakeConstants.kArmUpSpeed);
    //assume up is positive motor speed
    // if (Math.abs(error) > (Math.abs(setpoint) / 1.5)) {
    //   error = Math.abs(setpoint) - Math.abs(intake.getArmEncoder().getPosition());
    //   integral = integral + error;
    //   derivative =  error - lastError;
    //   motorPower = (kP * error) + (Ki * integral) + (Kd * derivative);

    //   if (motorPower > 0.65) {
    //     motorPower = 0.65;
    //   }

    //   if (motorPower < 0.20) {
    //     motorPower = 0;
    //   }

    //   if (setpoint < 0) {
    //     motorPower = -motorPower;
    //   }

    //   if (motorPower == 0) {
    //     done = true;
    //   }

    //   System.out.println(error);

    //   intake.setArmSpeed(motorPower);
    // }

    // else {
    //   done = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intake.getArmVoltage()>15){
      return true;
    }
    return false;
  }
}
