// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber_Left_Subsystem;

public class ClimberLeftRun extends Command {
  private final Climber_Left_Subsystem m_subsystem;
  private DoubleSupplier m_speed;
  /** Creates a new ClimberLeftRun. */
  public ClimberLeftRun(DoubleSupplier speed, Climber_Left_Subsystem subsystem) {
    m_speed = speed;
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = Math.copySign(m_speed.getAsDouble() * m_speed.getAsDouble(), m_speed.getAsDouble());
    m_subsystem.my_ClimberRun(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.my_ClimberRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
