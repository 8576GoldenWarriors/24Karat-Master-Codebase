// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterRoller;

public class ReverseShoot extends Command {
  /** Creates a new ReverseShoot. */
public ShooterRoller shooterRoller;
  /** Creates a new Shoot. */

  public BangBangController controller = new BangBangController();

  

  public ReverseShoot(ShooterRoller shooterRoller) {
    this.shooterRoller = shooterRoller;

    shooterRoller.setRevved(true);

    shooterRoller.setAmping(false);
    addRequirements(shooterRoller);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!RobotContainer.operatorController.getHID().getLeftBumper()){
      shooterRoller.setSpeed(-0.2);
      shooterRoller.setRevved(true);
    }
    else{
      shooterRoller.setSpeed(0);
      shooterRoller.setRevved(false);
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterRoller.setSpeed(0); //Sets the shooter to 0.2 as its idle mode instead of not moving at all.

    shooterRoller.setRevved(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return (RobotContainer.operatorController.getHID().getBButtonPressed() && !RobotContainer.operatorController.getHID().getLeftBumper()) ? true : false;

  }
}
