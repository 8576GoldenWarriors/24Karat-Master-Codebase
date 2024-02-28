// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class HoldShooter extends Command {
  /** Creates a new HoldShooter. */
  Shooter shooter;
  public HoldShooter(Shooter shooter) {

    this.shooter = shooter;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((RobotContainer.operatorController.getHID().getYButtonPressed() && RobotContainer.operatorController.getHID().getLeftBumperPressed())
    || (RobotContainer.operatorController.getHID().getBButtonPressed() && RobotContainer.operatorController.getHID().getLeftBumperPressed())
    ){
      return true;
    }
    return false;
  }
}
