// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
// import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhasingLEDPattern;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Drivetrain drivetrain = Drivetrain.getInstance();

  private RobotContainer m_robotContainer;

  //private UsbCamera camera;

//   public static final AddressableLED m_led = new AddressableLED(Constants.LEDConstants.LED_PORT1);
//  public static final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LedLength1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /*camera = CameraServer.startAutomaticCapture(4);

    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    camera.setVideoMode(PixelFormat.kMJPEG, 240, 240, 20);*/
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_led.setLength(Constants.LEDConstants.LedLength1);
    // for(int i = 0; i<Constants.LEDConstants.LedLength1; i++){
    //   m_ledBuffer.setHSV(i, 126, 100, 100);
    // }
    // m_led.setData(m_ledBuffer);

    // m_led.start();
    
    m_robotContainer = new RobotContainer();
   
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
  //   if(RobotContainer.m_IntakeRoller.getDigitalInput().get()){
  //     for(var i=0; i<Constants.LEDConstants.LedLength1; i++){
  //       //gold. no note
  //       m_ledBuffer.setHSV(i, 45, 100, 100);
  //     }
      
  //   }
  //   else{
  //     for(var i=0; i<Constants.LEDConstants.LedLength1; i++){
  //       //green. note intaked
  //       m_ledBuffer.setHSV(i, 126, 69, 100);
  //     }
  //  }
  //   m_led.setData(m_ledBuffer);

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
     m_robotContainer.ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(255, 90, 0), 0.5));
    //RobotContainer.m_Climber.setCoast();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    drivetrain.resetAllEncoders();
    drivetrain.setAllIdleMode(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    else{
      RobotContainer.m_Intake.zeroEncoder();
    }
    drivetrain.zeroHeading();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    drivetrain.resetAllEncoders();
    drivetrain.setAllIdleMode(true);

    RobotContainer.m_Shooter.zeroEncoder();

    //RobotContainer.m_ShooterRoller.setSpeed(0.2);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
