// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeRoller;

public class IntakeOut extends Command {
  private IntakeRoller intakeRoller;

  //public BangBangController controller = new BangBangController();
  
  /** Creates a new IntakeOut. */
  public IntakeOut(IntakeRoller intakeRoller) {
    this.intakeRoller = intakeRoller;

    addRequirements(intakeRoller);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.operatorController.getHID().getLeftBumper()) {
      intakeRoller.setRollerSpeed(Constants.IntakeConstants.kRollerOutSpeed);
    }
    else{
      intakeRoller.setRollerSpeed(0);
    }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeRoller.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
