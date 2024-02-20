package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_intakeMotor;
final double inputspeed= .5;
final double outputspeed= -.5;
final String speedinput="intake speed";
final String speedoutput="outake speed";
private double setPoint;
private double backUp;
private String Key;

  public IntakeSubsystem() {
    
    m_intakeMotor = new TalonFX(Constants.MyCANID.intake);

  }

  @Override
  public void periodic() {

  }

  public void runIntakeMotor() {
    backUp = inputspeed;
    Key = speedinput;
    setPoint = getPreferencesDouble(Key, backUp);
    
    m_intakeMotor.set(setPoint);
  }

  public void feedNote() {
    backUp = outputspeed;
    Key = speedoutput;
    setPoint = getPreferencesDouble(Key, backUp);

    m_intakeMotor.set(setPoint);
  }

  private Double getPreferencesDouble(String key, double backup){
    if (!Preferences.containsKey(key)){
      Preferences.initDouble(key, backup);
      Preferences.setDouble(key, backup);
    }
  return Preferences.getDouble(key, backup);
  }
}
