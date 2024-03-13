// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterUp extends Command {
  /** Creates a new ShooterUp. */
  Shooter shooter;
  public ShooterUp(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setPivotSpeed(Constants.ShooterConstants.kPivotUpSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPivotSpeed(0);
    //shooter.setShooterAngle(shooter.getShooterEncoder().get());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooter.getPivotMotorVoltage()>35){
      return true;
    }
    return false;
  }
}
