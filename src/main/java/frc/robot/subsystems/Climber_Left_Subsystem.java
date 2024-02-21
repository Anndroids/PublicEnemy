// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber_Left_Subsystem extends SubsystemBase {
  private final TalonFX m_motor;


  
  /** Creates a new Climber_Left_Subsystem. */
  public Climber_Left_Subsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    m_motor = new TalonFX(Constants.MyCANID.climberLeft);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void my_ClimberRun(double speed){
    m_motor.set(speed);
  }
}
