package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

private final TalonFX m_intakeMotor;
private DigitalInput m_LimitSwitch;


  public IntakeSubsystem() {
    
    m_intakeMotor = new TalonFX(Constants.MyCANID.intake);

    m_LimitSwitch = new DigitalInput(1);
  

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake LimitSwitch", getLimitSwitch());

  }

  public void my_MotorRun(double speed){
    m_intakeMotor.set(speed);
  }

  public void stop(){
    m_intakeMotor.set(0);
  }





  public boolean getLimitSwitch(){
    return !m_LimitSwitch.get();
  }
}
