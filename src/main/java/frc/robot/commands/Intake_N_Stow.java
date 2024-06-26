// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Intakewrist_MM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake_N_Stow extends SequentialCommandGroup {
  /** Creates a new Intake_N_Stow. */
  public Intake_N_Stow(IntakeSubsystem m_intakeSubsystem, Intakewrist_MM m_intakewrist_MM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                new IntakeSubsystem_Run_UntillSwitch(.5, m_intakeSubsystem),
                new IntakeWrist_To_Setpoint(()-> 0, m_intakewrist_MM)




    );
  }
}
