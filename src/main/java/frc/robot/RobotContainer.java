// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberLeftRun;
import frc.robot.commands.ClimberRightRun;
import frc.robot.commands.Intake_Feed_Note;
import frc.robot.commands.Shooter_Run;
import frc.robot.commands.TeleOpCommand;
import frc.robot.subsystems.Climber_Left_Subsystem;
import frc.robot.subsystems.Climber_Right_Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
 
public class RobotContainer {
  public static final XboxController m_driver = new XboxController(0);
  public static final XboxController m_operator = new XboxController(1);

  //Subsytems:
  public static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final Climber_Left_Subsystem m_climber_Left_Subsystem = new Climber_Left_Subsystem();
  public static final Climber_Right_Subsystem m_climber_Right_Subsystem = new Climber_Right_Subsystem();

  public RobotContainer() {    
    
    m_drivetrain.setDefaultCommand(new TeleOpCommand( () -> {return m_driver.getRawAxis(1);}, 
                                                      () -> {return m_driver.getRawAxis(0);}, 
                                                      () -> {return m_driver.getRawAxis(4);}, 
                                                      m_drivetrain));
    
    
  // m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.c_varShoot( 0.9*(m_operator.getRawAxis(2)), 0.7*(m_operator.getRawAxis(2))), m_shooterSubsystem));

  
  configureBindings();

  
  }

  private void configureBindings() {
      
      Trigger l_Trigger = new Trigger(()-> m_operator.getRawAxis(2) > .2);
      l_Trigger.whileTrue(new Shooter_Run(()-> 0.3, ()->0.2, m_shooterSubsystem));
      
      Trigger r_Trigger = new Trigger(()-> m_operator.getRawAxis(3) > .2);
      r_Trigger.whileTrue(new Intake_Feed_Note(m_intakeSubsystem));

      Trigger leftClimber = new Trigger(()->m_operator.getLeftBumper()).and(()-> Math.abs(m_operator.getRawAxis(1))>0.1);
      leftClimber.whileTrue(new ClimberLeftRun(()-> -m_operator.getRawAxis(1), m_climber_Left_Subsystem));
      
      Trigger rightClimber = new Trigger(()->m_operator.getLeftBumper()).and(()-> Math.abs(m_operator.getRawAxis(5))>0.1);
      rightClimber.whileTrue(new ClimberRightRun(()-> -m_operator.getRawAxis(5), m_climber_Right_Subsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  

}
