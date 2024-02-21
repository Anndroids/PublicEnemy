// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Shooter_Run extends Command {
  private final ShooterSubsystem m_subsystem;
  private DoubleSupplier m_leftSpeed;
  private DoubleSupplier m_rightSpeed;

  /** Creates a new Shooter_Run. */
  public Shooter_Run(DoubleSupplier leftspeed, DoubleSupplier rightspeed, ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    m_leftSpeed = leftspeed;
    m_rightSpeed = rightspeed;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.c_varShoot(m_rightSpeed.getAsDouble(), m_leftSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.c_varShoot(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
