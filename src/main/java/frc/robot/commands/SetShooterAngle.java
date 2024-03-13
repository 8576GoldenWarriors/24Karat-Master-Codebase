// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SetShooterAngle extends Command {
  PIDController controller;
  Shooter shooter;
  double desiredAngle;
  /** Creates a new ShooterMid. */
  public SetShooterAngle(Shooter shooter, double desiredAngle) {
    this.shooter = shooter;
    this.desiredAngle = desiredAngle;

    controller = new PIDController(0.5, 0, 0.1);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setPivotSpeed(controller.calculate(shooter.getAbsoluteDistance(), desiredAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(shooter.getShooterEncoder().getDistance()-desiredAngle)<0.00555556){
      return true;
    }
    return false;
  }
}
