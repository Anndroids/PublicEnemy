// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.TeleOpCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
 
public class RobotContainer {
  public static final XboxController m_driver = new XboxController(0);
  public static final XboxController m_operator = new XboxController(1);
  public static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  public static final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {    m_drivetrain.setDefaultCommand(new TeleOpCommand(() -> {return m_driver.getRawAxis(1);}, () -> {return m_driver.getRawAxis(0);}, () -> {return m_driver.getRawAxis(4);}, m_drivetrain));
    
  // m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.c_varShoot( 0.9*(m_operator.getRawAxis(2)), 0.7*(m_operator.getRawAxis(2))), m_shooterSubsystem));

  
  configureBindings();

  
  }

  private void configureBindings() {


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  

}
