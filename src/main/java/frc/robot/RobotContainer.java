// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.OverrideIntakeDown;
import frc.robot.commands.OverrideIntakeUp;
import frc.robot.commands.SetShooterAmp;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterDown;
import frc.robot.commands.ShooterUp;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.PhasingLEDPattern;
import frc.robot.subsystems.PhyscialLEDStrip;
import frc.robot.subsystems.RainbowLEDPattern;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRoller;


public class RobotContainer {

  public static final Drivetrain drivetrain = Drivetrain.getInstance();


  public static final Intake m_Intake = new Intake();
  public static final IntakeRoller m_IntakeRoller = new IntakeRoller();

  public static final Shooter m_Shooter = new Shooter();
  public static final ShooterRoller m_ShooterRoller = new ShooterRoller();
  public static final Climber m_Climber = new Climber();

  private final LEDStrip ledStrip;
 // public static final LED m_led = new LED(Constants.LEDConstants.LED_PORT1, Constants.LEDConstants.LedLength1);

 



  public static final CommandXboxController driverController = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    ledStrip = new PhyscialLEDStrip(9, 58);

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

    ledStrip.setDefaultCommand(new RunCommand(() -> {
      if (!m_IntakeRoller.getDigitalInput().get()){
        ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(44, 255,10), 0.5));
      }
      else{
        if (m_Climber.getRainbowBoolean()){
          ledStrip.usePattern(new RainbowLEDPattern(9, 7));
        }
        else{
        //ledStrip.usePattern(new RainbowLEDPattern(8, 2));
        ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(255, 0, 0
        ), 0.5));
        }
      }



    }, ledStrip)); 
      
    //Operator Controller

     // Intake
    //operatorController.a().whileTrue(new IntakeUp(m_Intake));
    // operatorController.a().whileTrue(new IntakeDown(m_Intake));//new SequentialCommandGroup(new IntakeIn(m_Intake), new IntakeUp(m_Intake), new Shoot(m_Shooter)));
    // operatorController.x().whileTrue(new IntakeUp(m_Intake));

    operatorController.a().whileTrue(new IntakeIn(m_IntakeRoller));
    operatorController.x().whileTrue(new IntakeOut(m_IntakeRoller));


  driverController.povDown().whileTrue(new OverrideIntakeUp(m_Intake));
  driverController.povUp().whileTrue(new OverrideIntakeDown(m_Intake));

    //Shooter
  
    operatorController.y().onTrue(new Shoot(m_ShooterRoller)); //b button ends shoot command, defined in shoot command
    //Shintake
    // operatorController.povDown().whileTrue(new Shintake(m_Shooter));
    
    //Climber
    //Window button is button #7. Retracts the climber.
    // operatorController.back().whileTrue(new Climb(m_Climber));
    driverController.b().whileTrue(new Climb(m_Climber));
    // //Three line button is button #8. Extends the climber.
    driverController.a().whileTrue((new ClimbDown(m_Climber)));

    driverController.rightBumper().whileTrue(new IntakeOut(m_IntakeRoller));

    driverController.button(7).onTrue(new InstantCommand(() -> m_Shooter.zeroEncoder()));
    // operatorController.start().whileTrue(new ClimbDown(m_Climber));

    //driverController.y().onTrue(getAutonomousCommand())
    
    //Arm
    //Commented bindings match the documented bindings
    operatorController.leftBumper().and(operatorController.a()).whileTrue( new  OverrideIntakeDown(m_Intake));
    operatorController.leftBumper().and(operatorController.x()).whileTrue( new OverrideIntakeUp(m_Intake));
    //operatorController.a().and(operatorController.leftBumper()).onTrue(new IntakeUp(m_Intake));
    //operatorController.x().and(operatorController.leftBumper()).onTrue(new IntakeDown(m_Intake));

    //test code for shooter pivot
    operatorController.leftBumper().and(operatorController.b()).whileTrue( new ShooterDown(m_Shooter));
    operatorController.leftBumper().and(operatorController.y()).whileTrue( new ShooterUp(m_Shooter));
    

    operatorController.povLeft().onTrue(new SetShooterAngle(m_Shooter, 0.015));
    operatorController.povUp().onTrue(new SetShooterAngle(m_Shooter, 0.07));
    operatorController.povRight().onTrue(new SetShooterAmp(m_Shooter, 0.096, m_ShooterRoller));


    
     
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
    NamedCommands.registerCommand("ResetHeading", (new InstantCommand(() -> drivetrain.zeroHeading())).deadlineWith(new InstantCommand(() ->  new WaitCommand(1))));

    NamedCommands.registerCommand("StopModules", (new InstantCommand(() -> drivetrain.stopModules())).deadlineWith(new InstantCommand(() ->  new WaitCommand(1))));
    //Shooter Commands:
    NamedCommands.registerCommand("RunShooter", (new InstantCommand(() -> m_ShooterRoller.setSpeed(Constants.ShooterConstants.kShooterSpeed))));
    NamedCommands.registerCommand("StopShooter", (new InstantCommand(() -> m_ShooterRoller.stopShooter())).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.2))));
    NamedCommands.registerCommand("ShooterMiddle", (new SetShooterAngle(m_Shooter, .016)));//new InstantCommand(() -> m_Shooter.setPivotSpeed(Constants.ShooterConstants.kPivotDownSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.5))));
    NamedCommands.registerCommand("ShooterUp", (new SetShooterAngle(m_Shooter, 0.07)).deadlineWith(new WaitCommand(2)));//new InstantCommand(() -> m_Shooter.setPivotSpeed(Constants.ShooterConstants.kPivotUpSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.5))));
    NamedCommands.registerCommand("ShooterDown", (new SetShooterAngle(m_Shooter, 0)).deadlineWith(new WaitCommand(2)));//new SetShooterAngle(m_Shooter, 0.07)));//new InstantCommand(() -> m_Shooter.setPivotSpeed(Constants.ShooterConstants.kPivotUpSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.5))));
    NamedCommands.registerCommand("StopShooterPivot", (new InstantCommand(() -> m_Shooter.setPivotSpeed(0))).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.5))));
    NamedCommands.registerCommand("ResetEncoders", (new InstantCommand(() -> m_Shooter.zeroEncoder()).deadlineWith(new InstantCommand(() -> new WaitCommand(0.5)))));
    NamedCommands.registerCommand("ShooterManualDown", (new InstantCommand(() -> m_Shooter.setPivotSpeed(0.4))).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.5))));
    NamedCommands.registerCommand("ShooterManualUp", (new InstantCommand(() -> m_Shooter.setPivotSpeed(-0.4))).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.5))));


    //Intake Commands:
    NamedCommands.registerCommand("IntakeUp", (new IntakeUp(m_Intake)));//new InstantCommand(() -> m_Intake.setArmSpeed(Constants.IntakeConstants.kArmUpSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(2))));;
    NamedCommands.registerCommand("IntakeDown", (new IntakeDown(m_Intake)));//new InstantCommand(() -> m_Intake.setArmSpeed(Constants.IntakeConstants.kArmDownSpeed))).deadlineWith(new InstantCommand(() ->  new WaitCommand(3))));
    NamedCommands.registerCommand("IntakeOut", (new InstantCommand(() -> m_IntakeRoller.setRollerSpeed(-0.95))).deadlineWith(new InstantCommand(() ->  new WaitCommand(3))));
    NamedCommands.registerCommand("IntakeIn", (new InstantCommand(() -> m_IntakeRoller.setRollerSpeed(0.95))).deadlineWith(new InstantCommand(() ->  new WaitCommand(1.5))));
    NamedCommands.registerCommand("StopIntake", (new InstantCommand(() -> m_IntakeRoller.stopRollerSpeed())).deadlineWith(new InstantCommand(() ->  new WaitCommand(0.2))));
    NamedCommands.registerCommand("IntakePIDReset", (new InstantCommand(() -> m_Intake.zeroEncoder())));
  }
}