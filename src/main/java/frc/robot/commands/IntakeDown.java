// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeDown extends Command {
  private Intake intake;
  private DutyCycleEncoder encoder;

  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(8.75, 4.55);
  private ProfiledPIDController controller = new ProfiledPIDController(.9, 0.09, 0.001, constraints);

  // double setpoint;
  // double kP;
  // double Ki;
  // double Kd;
  double error;
  double lastError;
  // double integral;
  // double derivative;
  // double avgPos;
  // double motorPower;

  boolean done = false;

  double encoderDistance;


  public IntakeDown(Intake intake) {
    this.intake = intake;
    this.encoder = intake.getArmEncoder();

    encoderDistance = encoder.getDistance();


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
    // avgPos = intake.getArmEncoder().getDistance();
    // motorPower = 0;

    //error = Constants.IntakeConstants.kArmUpPosition - encoder.getDistance();
    // integral = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.setArmSpeed(Constants.IntakeConstants.kArmDownSpeed);
      controller.setGoal(Constants.IntakeConstants.kArmDownPosition);
      // if (encoder.getDistance() > 0.009 && encoder.getDistance() < 0.18){
      //   speed = speed * 15;
      // }

      if (encoderDistance <= 0){
        encoderDistance = 0;
      }

      intake.setArmSpeed(controller.calculate(encoderDistance));

    //assume up is positive motor speed
    
    // if(!(Math.abs(error) > (Math.abs(Constants.IntakeConstants.kArmUpPosition) / 1.5))){
    //   done = true;
    // }
    error = Constants.IntakeConstants.kArmUpPosition - encoderDistance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(done){
      return true;
    }    
    return false;
  }
}
