// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveTrainSetHeading;
import frc.robot.commands.Intake_Feed_Note;
import frc.robot.commands.Shooter_Run;
import frc.robot.commands.TeleOpCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto06_BLUE_Amp_Shoot_N_Drive extends SequentialCommandGroup {
  /** Creates a new Auto06_BLUE_amp_Shoot_N_Drive. */
  public Auto06_BLUE_Amp_Shoot_N_Drive(ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem, DrivetrainSubsystem m_drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveTrainSetHeading(60, m_drivetrain),
      new Auto_Shoot_PreLoad_Center(m_shooterSubsystem,m_intakeSubsystem),
      new TeleOpCommand(() ->-.25,() ->-0.15,() ->-0.0,m_drivetrain).withTimeout(4)
    );
  }
}
