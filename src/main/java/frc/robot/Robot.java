// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib14.MCRCommand;
import frc.robot.subsystems.ElevatorSubsystem;
/*
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
import frc.robot.subsystems.ElevatorSubsystem;

public class Robot extends TimedRobot {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final XboxController operator = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
  
    // private final Trigger crawl = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    // private final Trigger sprint = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  
    /* Operator Controls */
    private final Trigger intakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final Trigger shooterPosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    public boolean intakeStatus = false;    
    // private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final Trigger shooterTrigger = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    // private final JoystickButton armWrist = new JoystickButton(operator, XboxController.Button.kA.value);

    MCRCommand autoMission;

    /* Subsystems */
    // private final Swerve s_Swerve = new Swerve();
    // private final IntakeSubsystem m_IntakeSubsystem = IntakeSubsystem.getInstance();
    // private final NoteTransitSubsystem m_NoteTransitSubsystem = NoteTransitSubsystem.getInstance();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    /* autos */
    MCRCommand twoNoteCenter;

      SendableChooser<Command> autoChooser ;

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
//38 container
//84 swerve
 
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // climb.zeroEncoder();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // s_Swerve.periodicValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // s_Swerve.zeroGyro();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // driver.getRawButton(XboxController.Button.kX.value).onTrue(climb.setTargetLocation(climb.kFeederStation));
    // driver.getRawButton(XboxController.Button.kB.value).onTrue(climb.setTargetLocation(20));
    // driver.button(XboxController.Button.kY.value).onTrue(climb.setTargetLocation(0));
    // climb.setTargetLocation(40);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

    }
 
    // s_Swerve.zeroGyro();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (driver.getRawButton(XboxController.Button.kLeftBumper.value)) {
      elevatorSubsystem.setTargetLocation(Constants.L1_Distance);
   }
   if (driver.getRawButton(XboxController.Button.kY.value)) {
    elevatorSubsystem.setTargetLocation(Constants.L2_Distance);
   }
   if (driver.getRawButton(XboxController.Button.kB.value)) {
    elevatorSubsystem.setTargetLocation(Constants.L3_Distance);
  }
   if (driver.getRawButton(XboxController.Button.kA.value)) {
    elevatorSubsystem.setTargetLocation(Constants.L4_Distance);
  }
    if (driver.getRawButton(XboxController.Button.kRightBumper.value)) {
      elevatorSubsystem.setTargetLocation(0);
  }
 

  elevatorSubsystem.periodic();

    }
    // System.out.println(s_Swerve.getTotalDist());

    // callPeriodic();
    // s_Swerve.periodic(
    //   () -> -driver.getRawAxis(translationAxis), 
    //   () -> -driver.getRawAxis(strafeAxis), 
    //   () -> -driver.getRawAxis(rotationAxis), 
    //   () -> false /* Never Robot-Oriented */
    // );
  }

  @Override
  public void testInit() {}
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  


  public void callPeriodic(){

  }
}


