// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveTrainSetHeading extends InstantCommand {
  private final DrivetrainSubsystem m_Subsystem;
  private double m_setpoint;
  public DriveTrainSetHeading(double setpoint, DrivetrainSubsystem subsystem) {
    m_Subsystem = subsystem;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        //<RED ACTION>
        m_Subsystem.setGyroHeading(-m_setpoint);
      }
      if (ally.get() == Alliance.Blue) {
        //<BLUE ACTION>
        m_Subsystem.setGyroHeading(m_setpoint);
      }
    }else{
    //<NO COLOR YET ACTION>
    } 
  }
}
