// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoCommands.Auto01_Shoot_N_Drive;
import frc.robot.AutoCommands.Auto02_Shoot_N_Second_Note;

import frc.robot.AutoCommands.Auto03_RED_Source_Shoot_N_Drive;

import frc.robot.AutoCommands.Auto04_RED_Amp_Shoot_N_Drive;
import frc.robot.AutoCommands.Auto05_BLUE_Source_Shoot_N_Drive;
import frc.robot.AutoCommands.Auto06_BLUE_Amp_Shoot_N_Drive;
import frc.robot.AutoCommands.Auto_Shoot_PreLoad_Center;
import frc.robot.commands.ClimberLeftRun;
import frc.robot.commands.ClimberRightRun;
import frc.robot.commands.DriveTrainSetHeading;
import frc.robot.commands.IntakeSubsystem_Run;
import frc.robot.commands.IntakeSubsystem_Run_UntillSwitch;
import frc.robot.commands.IntakeWrist_MM_Reset;
import frc.robot.commands.IntakeWrist_To_Setpoint;
import frc.robot.commands.Intake_Feed_Note;
import frc.robot.commands.Intake_N_Stow;
import frc.robot.commands.Shooter_Run;
import frc.robot.commands.TeleOpCommand;
import frc.robot.subsystems.Climber_Left_Subsystem;
import frc.robot.subsystems.Climber_Right_Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Intakewrist_MM;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.CameraServerJNI;
 
public class RobotContainer {
  public static final XboxController m_driver = new XboxController(0);
  public static final XboxController m_operator = new XboxController(1);

  //Subsytems:
  public static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final Climber_Left_Subsystem m_climber_Left_Subsystem = new Climber_Left_Subsystem();
  public static final Climber_Right_Subsystem m_climber_Right_Subsystem = new Climber_Right_Subsystem();
  public static final Intakewrist_MM m_intakewrist = new Intakewrist_MM();
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {   
  
    SmartDashboard.putData(m_intakeSubsystem);
    SmartDashboard.putData(m_intakewrist);
    SmartDashboard.putNumber("wrist angle", 95);

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Do Nothing", new PrintCommand("Do Nothing"));
    m_chooser.addOption("shoot preloaded note", new Auto_Shoot_PreLoad_Center(m_shooterSubsystem, 
                                                                                  m_intakeSubsystem));
    m_chooser.addOption("Auto 01 Center Shoot N Drive", new Auto01_Shoot_N_Drive(m_shooterSubsystem, 
                                                                                      m_intakeSubsystem, 
                                                                                        m_drivetrain));
    m_chooser.addOption("Auto 02 Center Shoot N Second Note", new Auto02_Shoot_N_Second_Note(m_shooterSubsystem,
                                                                                                  m_intakeSubsystem,
                                                                                                  m_drivetrain,
                                                                                                  m_intakewrist));
    m_chooser.addOption("Auto 03 RED Source Shoot N Drive", new Auto03_RED_Source_Shoot_N_Drive(m_shooterSubsystem, 
                                                                                      m_intakeSubsystem, 
                                                                                        m_drivetrain));
    m_chooser.addOption("Auto 04 RED Amp Shoot N Drive", new Auto04_RED_Amp_Shoot_N_Drive(m_shooterSubsystem, 
                                                                                      m_intakeSubsystem, 
                                                                                        m_drivetrain));
    m_chooser.addOption("Auto 05 BLUE Source Shoot N Drive", new Auto05_BLUE_Source_Shoot_N_Drive(m_shooterSubsystem, 
                                                                                      m_intakeSubsystem, 
                                                                                        m_drivetrain));
    m_chooser.addOption("Auto 06 BLUE Amp Shoot N Drive", new Auto06_BLUE_Amp_Shoot_N_Drive(m_shooterSubsystem, 
                                                                                      m_intakeSubsystem, 
                                                                                        m_drivetrain));

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
    //SmartDashboard.putData("runintake", new IntakeSubsystem_Run_UntillSwitch(.3,m_intakeSubsystem));
    //SmartDashboard.putData("intakeandstow", new Intake_N_Stow(m_intakeSubsystem, m_intakewrist));
   
    
    m_drivetrain.setDefaultCommand(new TeleOpCommand( () -> {return m_driver.getRawAxis(1);}, 
                                                      () -> {return m_driver.getRawAxis(0);}, 
                                                      () -> {return m_driver.getRawAxis(4);}, 
                                                      m_drivetrain));
    
    
  // m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.c_varShoot( 0.9*(m_operator.getRawAxis(2)), 0.7*(m_operator.getRawAxis(2))), m_shooterSubsystem));

  
  configureBindings();

  
  }

  private void configureBindings() {
      Trigger ampshot = new Trigger(()-> m_operator.getRawAxis(2) > .2).and(() -> m_operator.getBButton());
      ampshot.whileTrue(new Shooter_Run(()-> 0.12, ()->0.12, m_shooterSubsystem));

      Trigger l_Trigger = new Trigger(()-> m_operator.getRawAxis(2) > .2).and(() -> !m_operator.getBButton());
      l_Trigger.whileTrue(new Shooter_Run(()-> 0.7, ()->0.5, m_shooterSubsystem));
      
      Trigger r_Trigger = new Trigger(()-> m_operator.getRawAxis(3) > .2);
      r_Trigger.whileTrue(new Intake_Feed_Note(m_intakeSubsystem));

      Trigger leftClimber = new Trigger(()->m_operator.getLeftBumper()).and(()-> Math.abs(m_operator.getRawAxis(1))>0.1);
      leftClimber.whileTrue(new ClimberLeftRun(()-> -m_operator.getRawAxis(1), m_climber_Left_Subsystem));
      
      Trigger rightClimber = new Trigger(()->m_operator.getLeftBumper()).and(()-> Math.abs(m_operator.getRawAxis(5))>0.1);
      rightClimber.whileTrue(new ClimberRightRun(()-> -m_operator.getRawAxis(5), m_climber_Right_Subsystem));

      Trigger wristToStow = new Trigger(() -> m_operator.getXButton()).or(() -> m_driver.getXButton());
      wristToStow.onTrue(new IntakeWrist_To_Setpoint(() ->Constants.IntakeVarialbles.STOW_POSITION, m_intakewrist));
      
      Trigger wristToPick = new Trigger(() -> m_operator.getAButton()).or(() -> m_driver.getAButton());
      wristToPick.onTrue(new IntakeWrist_To_Setpoint(() -> Constants.IntakeVarialbles.DEPLOY_POSITION, m_intakewrist));

      Trigger wristToAmp = new Trigger(() -> m_operator.getYButton()).or(() -> m_driver.getYButton());
      //wristToAmp.onTrue(new IntakeWrist_To_Setpoint(() -> SmartDashboard.getNumber("wrist angle", 95), m_intakewrist));
      wristToAmp.onTrue(new IntakeWrist_To_Setpoint(() -> Constants.IntakeVarialbles.AMP_POSITION, m_intakewrist));
      
      Trigger resetWrist = new Trigger(() -> m_operator.getRawButton(7)).and(() -> m_operator.getRawButton(8));
      resetWrist.onTrue(new IntakeWrist_MM_Reset(m_intakewrist));

      Trigger intakeNote = new Trigger(() -> m_operator.getRightBumper());
      //intakeNote.whileTrue(new IntakeSubsystem_Run(.30, m_intakeSubsystem));
      intakeNote.toggleOnTrue(new Intake_N_Stow(m_intakeSubsystem, m_intakewrist));
      
      Trigger resetgyro = new Trigger(() -> m_driver.getBackButtonPressed());
      resetgyro.onTrue(m_drivetrain.runOnce(m_drivetrain:: resetGyroHeading));
    }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
  

}
