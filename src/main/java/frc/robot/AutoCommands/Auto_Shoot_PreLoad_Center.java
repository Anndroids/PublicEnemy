// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake_Feed_Note;
import frc.robot.commands.Shooter_Run;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Shoot_PreLoad_Center extends SequentialCommandGroup {
  /** Creates a new Auto_Shoot_PreLoad. */
  public Auto_Shoot_PreLoad_Center(ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                new ParallelCommandGroup(
                                new Shooter_Run(()-> 0.7, ()->0.5, m_shooterSubsystem),
                                new SequentialCommandGroup(
                                                          new WaitCommand(1.5),    //Spin-up time
                                                          new Intake_Feed_Note(m_intakeSubsystem)  //Fire Note
                                )
                ).withTimeout(3)  //Total Shot Time
    );
  }
}
