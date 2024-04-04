// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveTrainSetHeading;
import frc.robot.commands.IntakeWrist_To_Setpoint;
import frc.robot.commands.Intake_Feed_Note;
import frc.robot.commands.Intake_N_Stow;
import frc.robot.commands.Shooter_Run;
import frc.robot.commands.TeleOpCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Intakewrist_MM;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Shoot_N_Third_Note_Left extends SequentialCommandGroup {
  /** Creates a new Auto_Shoot_N_Third_Note_Left. */
  public Auto_Shoot_N_Third_Note_Left(ShooterSubsystem m_shooterSubsystem, 
                                    IntakeSubsystem m_intakeSubsystem, 
                                    DrivetrainSubsystem m_drivetrain,
                                    Intakewrist_MM m_intakewrist_MM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveTrainSetHeading(0, m_drivetrain),
      new Auto_Shoot_PreLoad_Center(m_shooterSubsystem,m_intakeSubsystem), // Shoots Preload

      new IntakeWrist_To_Setpoint(()->Constants.IntakeVarialbles.DEPLOY_POSITION, m_intakewrist_MM), //Deploy Intake

      new WaitCommand(.25),  //Delay go Get Second Note

      new ParallelDeadlineGroup( //Drive Untill Drive Command is Complete
                                new TeleOpCommand(() ->-.2,() ->-0.0,() ->-0.0,m_drivetrain).withTimeout(2),
                                new Intake_N_Stow(m_intakeSubsystem,m_intakewrist_MM)
                                ),

      new IntakeWrist_To_Setpoint(()->Constants.IntakeVarialbles.STOW_POSITION, m_intakewrist_MM), //ReStow If not done above

      new TeleOpCommand(() ->.2,() ->0.0,() ->0.0,m_drivetrain).withTimeout(2), //Drive Back to the Speaker
     
      new Auto_Shoot_PreLoad_Center(m_shooterSubsystem,m_intakeSubsystem),

      new TeleOpCommand(() ->0.0, () ->-0.1, () ->0.0, m_drivetrain).withTimeout(2)                                                                                                         
    );
  }
}
