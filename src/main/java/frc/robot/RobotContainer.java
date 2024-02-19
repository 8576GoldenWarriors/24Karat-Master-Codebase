// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  public static final Drivetrain drivetrain = Drivetrain.getInstance();


  public static final Intake m_Intake = new Intake();
  public static final Shooter m_Shooter = new Shooter();
  public static final Climber m_Climber = new Climber();


  public static final CommandXboxController driverController = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    registerNamedCommands();

    configureBindings();

    drivetrain.setDefaultCommand(new SwerveDrive());

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoChooser", autoChooser);
  }


  private void configureBindings() {
    
    //Driver controller
    resetHeading_Start.onTrue(
      new InstantCommand(drivetrain::zeroHeading, drivetrain));

    //Operator Controller

     // Intake
    operatorController.a().whileTrue(new IntakeIn(m_Intake));
    operatorController.x().whileTrue(new IntakeOut(m_Intake));

    //Shooter
    operatorController.y().onTrue(new Shoot(m_Shooter)); //b button ends shoot command, defined in shoot command
    //operatorController.y().whileTrue(new Shoot(m_Shooter));  
    
    //Climber
    //Window button is button #7. Retracts the climber.
    operatorController.button(7).onTrue(new ClimbDown(m_Climber));
    //Three line button is button #8. Extends the climber.
    operatorController.button(8).onTrue(new Climb(m_Climber));
    
    //Arm
    //Commented bindings match the documented bindings
    operatorController.a().and(operatorController.leftBumper()).whileTrue( new IntakeUp(m_Intake) );
    operatorController.x().and(operatorController.leftBumper()).whileTrue( new IntakeDown(m_Intake) );
    //operatorController.a().and(operatorController.leftBumper()).onTrue(new IntakeUp(m_Intake));
    //operatorController.x().and(operatorController.leftBumper()).onTrue(new IntakeDown(m_Intake));
     
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("StopModules", new InstantCommand(() -> drivetrain.stopModules()));
  }
}