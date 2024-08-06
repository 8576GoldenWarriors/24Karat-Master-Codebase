// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Autospin extends Command {
   private Drivetrain drivetrain = Drivetrain.getInstance();

  double setAngle;
  double currentAngle;

  double frontSpeed;
  double sideSpeed;
  double rotationSpeed;

  double kP;
  double kI;
  double kD;

  PIDController rotationController;

  public Autospin(double angleDegrees) {

    setAngle = angleDegrees;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    kP = 1.0;
    kI = 0.0;
    kD = 0.0;

    rotationController = new PIDController(kP, kI, kD);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentAngle = RobotContainer.drivetrain.getHeading();

    frontSpeed = -RobotContainer.driverController.getLeftY() 
    * Math.abs(RobotContainer.driverController.getLeftY()) * 2.25;

    sideSpeed = -RobotContainer.driverController.getLeftX() 
    * Math.abs(RobotContainer.driverController.getLeftX()) * 2.25;

    rotationSpeed = rotationController.calculate(currentAngle, setAngle);


    RobotContainer.drivetrain.swerveDrive(
      frontSpeed, 
      sideSpeed, 
      rotationSpeed, 
      true, 
      new Translation2d(), 
      true
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.swerveDrive(
      frontSpeed, 
      sideSpeed, 
      0, 
      true, 
      new Translation2d(), 
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //if within 5 deg margin of error
    if(Math.abs(setAngle-currentAngle) < 5.0){ 
      return true;
    }

    //or if driver cancels rotation with rotation joystick input > 0.3
    else if(Math.abs(RobotContainer.driverController.getRightX()) > 0.3 
    || Math.abs(RobotContainer.driverController.getRightY())>0.3){
      return true;
    }

    return false;
  }
}
