// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeUp extends Command {
  /** Creates a new IntakeUp. */
  private Intake intake;
  private Encoder encoder;

  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5, 3);
  private ProfiledPIDController controller = new ProfiledPIDController(.001, 0, 0, constraints);

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

  public IntakeUp(Intake intake) {
    this.intake = intake;
    this.encoder = intake.getArmEncoder();

    encoderDistance = encoder.getDistance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
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

    // if (encoderDistance <= 0){
    //   encoderDistance = 0;
    // }

   // intake.setArmSpeed(Constants.IntakeConstants.kArmUpSpeed);
    controller.setGoal(Constants.IntakeConstants.kArmUpPosition);
    intake.setArmSpeed(controller.calculate(Math.abs(encoder.getDistance())));

  
   
    // if(!(Math.abs(error) > (Math.abs(Constants.IntakeConstants.kArmUpPosition) / 1.5))){
    //   done = true;
    // }
    if (encoder.getDistance() > -20){
      done = true;
    }
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
