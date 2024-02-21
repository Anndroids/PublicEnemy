// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intakewrist_MM;

public class IntakeWrist_To_Setpoint extends Command {
  private final Intakewrist_MM m_subsystem;
  private double m_setpoint;
  /** Creates a new IntakeWrist_To_Setpoint. */
  public IntakeWrist_To_Setpoint(double setpoint, Intakewrist_MM subsytem) {
    m_setpoint= setpoint;
    m_subsystem = subsytem;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.my_Wrist_MM(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
