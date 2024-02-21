package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_leftShooterMotor;
  private final TalonFX m_rightShooterMotor;

  public ShooterSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    m_leftShooterMotor = new TalonFX(Constants.MyCANID.leftShooter);
    m_rightShooterMotor = new TalonFX(Constants.MyCANID.rightShooter);

    m_leftShooterMotor.getConfigurator().apply(config);
    m_rightShooterMotor.getConfigurator().apply(config);

    m_leftShooterMotor.setInverted(false);
    m_rightShooterMotor.setInverted(false);
  }

  @Override
  public void periodic() {}

  public void c_subShoot() {
    m_leftShooterMotor.set(0.9);
    m_rightShooterMotor.set(0.7);
  }

    public void c_stopShooter() {
    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
  }

    public void c_varShoot(double rightSpeed, double leftSpeed) {
    m_leftShooterMotor.set(leftSpeed);
    m_rightShooterMotor.set(rightSpeed);
  }
}
