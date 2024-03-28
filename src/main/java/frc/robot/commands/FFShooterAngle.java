// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRoller;

public class FFShooterAngle extends Command {
  PIDController controller;
  ArmFeedforward ffcontroller;
  Shooter shooter;
  double desiredAngle;

  /** Creates a new FFShooterAngle. */

  public FFShooterAngle(Shooter shooter, double desiredAngle) {

    this.shooter = shooter;

    this.desiredAngle = desiredAngle;

    this.controller = new PIDController(6.0, 2.0, 0.001);

    this.ffcontroller = new ArmFeedforward(0, 0.15, 0.15, 0); //old ks = 0, old kg = 0.58, old kv = 2.53, old ka = 0.03

    addRequirements(shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



    double motorPower = (controller.calculate((shooter.getAbsoluteDistance()), (desiredAngle)));// + (ffcontroller.calculate(desiredAngle, 2)) / 14);
    System.out.println(motorPower);
    shooter.getShooterMotor().set(motorPower);

    //SmartDashboard.putNumber("Feed Forward: ", (ffcontroller.calculate(desiredAngle, 2)) / 14);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(shooter.getShooterEncoder().getDistance()-desiredAngle)<0.01 || Math.abs(shooter.getShooterEncoder().getDistance())>0.13){
      return true;
    }
    return false;
  }
}
