// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  public Shooter Shooter;
  /** Creates a new Shoot. */

  public BangBangController controller = new BangBangController();

  public Shoot(Shooter Shooter) {
    this.Shooter = Shooter;

    addRequirements(Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.setSpeed(Constants.ShooterConstants.kShooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return (RobotContainer.operatorController.getHID().getBButtonPressed()) ? true : false;

  }
}
