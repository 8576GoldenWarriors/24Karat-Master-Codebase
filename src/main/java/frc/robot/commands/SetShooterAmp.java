// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRoller;

public class SetShooterAmp extends Command {
  PIDController controller;
  Shooter shooter;
  ShooterRoller shooterRoller;
  double desiredAngle;
  /** Creates a new ShooterMid. */
  public SetShooterAmp(Shooter shooter, double desiredAngle, ShooterRoller shooterRoller) {
    this.shooter = shooter;
    this.desiredAngle = desiredAngle;
    this.shooterRoller = shooterRoller;

    controller = new PIDController(3.00, 6.00, 0.001);
    addRequirements(shooter, shooterRoller);
  }
  

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motorPower = controller.calculate(shooter.getAbsoluteDistance(), desiredAngle);
    shooter.setPivotSpeed(motorPower);
    shooterRoller.setSpeed(0.14);
    SmartDashboard.putNumber("Shooter PID Power", motorPower);
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
