// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intakewrist_MM extends SubsystemBase {
  private final TalonFX m_fx = new TalonFX(Constants.MyCANID.wristParent, "Rio");
  private final TalonFX m_fx_Follower = new TalonFX(Constants.MyCANID.wristChild, "Rio");
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private double scale = 360;

  /** Creates a new intakewrist. */
  public Intakewrist_MM() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 6; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 1.5; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    mm.MotionMagicJerk = 50;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 60;
    slot0.kI = 0;
    slot0.kD = 1;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 20;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    m_fx_Follower.setControl(new Follower(m_fx.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getPositionDeg());
    // This method will be called once per scheduler run
  }

  public void my_Wrist_MM(double deg){
    deg = MathUtil.clamp(deg, 0, 210);
    System.out.print(deg);
    m_fx.setControl(m_mmReq.withPosition(deg/scale).withSlot(0));
  }

  public double getPositionDeg(){
    return  m_fx.getPosition().getValueAsDouble() * scale;
  }

  public void resetWrist(){
      m_fx.setPosition(0);
  }
}
