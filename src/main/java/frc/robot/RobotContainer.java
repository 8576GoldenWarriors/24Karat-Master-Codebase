// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.Shintake;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterDown;
import frc.robot.commands.ShooterUp;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
   //Add all the choise of Autonomous modes to the Smart Dashboard
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
    //Shintake
    operatorController.povDown().whileTrue(new Shintake(m_Shooter));
    
    //Climber
    //Window button is button #7. Retracts the climber.
    //operatorController.button(7).onTrue(new ClimbDown(m_Climber));
    operatorController.back().whileTrue(new Climb(m_Climber));
    //Three line button is button #8. Extends the climber.
    //operatorController.button(8).onTrue(new Climb(m_Climber));
    operatorController.start().whileTrue(new ClimbDown(m_Climber));
    
    //Arm
    //Commented bindings match the documented bindings
    operatorController.a().and(operatorController.leftBumper()).whileTrue( new IntakeUp(m_Intake) );
    operatorController.x().and(operatorController.leftBumper()).whileTrue( new IntakeDown(m_Intake) );
    //operatorController.a().and(operatorController.leftBumper()).onTrue(new IntakeUp(m_Intake));
    //operatorController.x().and(operatorController.leftBumper()).onTrue(new IntakeDown(m_Intake));

    //test code for shooter pivot
    operatorController.b().and(operatorController.leftBumper()).whileTrue( new ShooterDown(m_Shooter));
    operatorController.y().and(operatorController.leftBumper()).whileTrue( new ShooterUp(m_Shooter));


    
     
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  //Autonomous Commands:
  public void registerNamedCommands(){
    //Drivetrain Commands:
    NamedCommands.registerCommand("StopModules", (new InstantCommand(() -> drivetrain.stopModules())).deadlineWith(new InstantCommand(() ->  new WaitCommand(1))));
    //Shooter Commands:
    NamedCommands.registerCommand("RunShooter", (new InstantCommand(() -> m_Shooter.setSpeed(Constants.ShooterConstants.kShooterSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(1))));
    NamedCommands.registerCommand("StopShooter", (new InstantCommand(() -> m_Shooter.stopShooter())).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.2))));
    NamedCommands.registerCommand("ShooterDown", (new InstantCommand(() -> m_Shooter.setPivotSpeed(Constants.ShooterConstants.kPivotDownSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.5))));;

    //Intake Commands:
    NamedCommands.registerCommand("IntakeUp", (new InstantCommand(() -> m_Intake.setArmSpeed(Constants.IntakeConstants.kArmUpSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(2))));;
    NamedCommands.registerCommand("IntakeDown", (new InstantCommand(() -> m_Intake.setArmSpeed(Constants.IntakeConstants.kArmDownSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(2))));
    NamedCommands.registerCommand("IntakeOut", (new InstantCommand(() -> m_Intake.setRollerSpeed(Constants.IntakeConstants.kRollerOutSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(2))));
    NamedCommands.registerCommand("StopIntakeOut", (new InstantCommand(() -> m_Intake.stopRollerSpeed())).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.2))));
  }
}