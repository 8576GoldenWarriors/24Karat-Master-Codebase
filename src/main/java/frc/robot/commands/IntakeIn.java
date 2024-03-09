// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeIn extends Command {
  public Intake intake;

  /** Creates a new Intake. */
  public IntakeIn(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.SetCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (intake.getDigitalInput().get()) {
    intake.setRollerSpeed(Constants.IntakeConstants.kRollerInSpeed);
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(intake.getDigitalInput().get()==true){
    //   return true;
    // }
    return false;
  }
}
