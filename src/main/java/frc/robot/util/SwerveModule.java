package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModule {
    public static class SwerveModuleConfig {
        public int TURN_MOTOR_ID;
        public double TURN_MOTOR_GEARING;
        public double TURN_CONTROLLER_P;
        public double TURN_CONTROLLER_D;
        public double TURN_CONTROLLER_KS;
        public double TURN_CONTROLLER_KV;
        public boolean TURN_INVERSE;
        public boolean TURN_ENCODER_INVERSE;
        public double TURN_ENCODER_OFFSET;
        public int TURN_ENCODER_ID;

        public int DRIVE_MOTOR_ID;
        public double DRIVE_MOTOR_GEARING;
        public double DRIVE_CONTROLLER_P;
        public double DRIVE_CONTROLLER_D;
        public double DRIVE_CONTROLLER_KS;
        public double DRIVE_CONTROLLER_KV;
        public boolean DRIVE_INVERSE;

        public double WHEEL_DIAMETER;

        public Translation2d MODULE_LOCATION = new Translation2d();
    }
    // CTRE
    private final TalonFX m_turnCTRE;
    private final TalonFXConfigurator m_turnConfigurator;
    private final TalonFXConfiguration m_turnConfig;

    private final TalonFX m_driveCTRE;
    private final TalonFXConfigurator m_driveConfigurator;
    private final TalonFXConfiguration m_driveConfig;
    private final StatusSignal<Double> m_driveVelocity;
    private final StatusSignal<Double> m_drivePosition;

    private final AnalogEncoder m_turnAnalogEncoder;

    private final VelocityVoltage m_velocityCommand;
    private final PositionVoltage m_positionCommand;

    public final Translation2d m_location;
    private final SwerveModuleConfig m_config;

    private final VoltageOut m_driveVoltage = new VoltageOut(10);
    private final VoltageOut m_turnVoltage = new VoltageOut(10);

    public SwerveModule(SwerveModuleConfig config) {
        // CTRE
        m_turnCTRE = new TalonFX(config.TURN_MOTOR_ID);
        m_turnConfigurator = m_turnCTRE.getConfigurator();
        m_turnConfig = new TalonFXConfiguration();
        m_turnConfig.MotorOutput.Inverted = config.TURN_INVERSE ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        m_turnConfig.Feedback.SensorToMechanismRatio = config.TURN_MOTOR_GEARING;
        m_turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // Change to RotorSensor if no CANcoder, or fused if using Phoenix Pro
        m_turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        m_turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_turnConfig.Slot0.kP = config.TURN_CONTROLLER_P;
        m_turnConfig.Slot0.kD = config.TURN_CONTROLLER_D;
        m_turnConfig.Slot0.kS = config.TURN_CONTROLLER_KS;
        m_turnConfig.Slot0.kV = config.TURN_CONTROLLER_KV;

        m_driveCTRE = new TalonFX(config.DRIVE_MOTOR_ID);
        m_driveConfigurator = m_driveCTRE.getConfigurator();
        m_driveConfig = new TalonFXConfiguration();
        m_driveConfig.MotorOutput.Inverted = config.DRIVE_INVERSE ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        m_driveConfig.Feedback.SensorToMechanismRatio = config.DRIVE_MOTOR_GEARING;
        m_driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_driveConfig.CurrentLimits.StatorCurrentLimit = 80;
        m_driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_driveConfig.Slot0.kP = config.DRIVE_CONTROLLER_P;
        m_driveConfig.Slot0.kD = config.DRIVE_CONTROLLER_D;
        m_driveConfig.Slot0.kS = config.DRIVE_CONTROLLER_KS;
        m_driveConfig.Slot0.kV = config.DRIVE_CONTROLLER_KV;
        m_driveConfigurator.apply(m_driveConfig);
        m_driveVelocity = m_driveCTRE.getVelocity();
        m_drivePosition = m_driveCTRE.getPosition();

        m_velocityCommand = new VelocityVoltage(10);
        m_velocityCommand.Slot = 0;
        m_positionCommand = new PositionVoltage(10);
        m_positionCommand.Slot = 0;

        m_turnAnalogEncoder = new AnalogEncoder(config.TURN_ENCODER_ID);
        m_turnCTRE.setPosition(m_turnAnalogEncoder.getAbsolutePosition() - config.TURN_ENCODER_OFFSET);
        m_turnConfigurator.apply(m_turnConfig);
        m_turnAnalogEncoder.setDistancePerRotation(2 * Math.PI);

        //SmartDashboard.putNumber(config.DRIVE_MOTOR_ID + "_Drive_Voltage", 10);
        //SmartDashboard.putNumber(config.TURN_MOTOR_ID + "_Turn_Voltage", 10);

        m_location = config.MODULE_LOCATION;
        m_config = config;
    }

    public void loop() {
        /*m_driveVoltage.Output = SmartDashboard.getNumber(m_config.DRIVE_MOTOR_ID + "_Drive_Voltage", 10);
        m_turnVoltage.Output = SmartDashboard.getNumber(m_config.TURN_MOTOR_ID + "_Turn_Voltage", 10);
        SmartDashboard.putNumber(m_config.TURN_ENCODER_ID + "_Heading", getTurnHeading());
        m_driveCTRE.setControl(m_driveVoltage);
        m_turnCTRE.setControl(m_turnVoltage);*/

        SmartDashboard.putNumber(m_config.TURN_MOTOR_ID + "_ROT", getTurnHeading());
        SmartDashboard.putNumber(m_config.TURN_MOTOR_ID + "_ROT_ABSOLUTE", m_turnAnalogEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(m_config.DRIVE_MOTOR_ID + "_SPEED", getDriveVelocity());
    }

    public double getTurnHeading() {
        return m_turnCTRE.getPosition().getValue();
    }

    public double getDriveVelocity() {
        double velocity;
        double returnValue;

        // CTRE
        m_driveVelocity.refresh();
        velocity = m_driveVelocity.getValue().doubleValue();
        returnValue = velocity * Math.PI * m_config.WHEEL_DIAMETER;
        
        return returnValue;
    }

     /**
     * Gets the drive position, in meters
     * @return wheel position in meters
     */
    public double getDrivePosition() {
        m_drivePosition.refresh();
        double position = m_drivePosition.getValue().doubleValue();
        return position * Math.PI * m_config.WHEEL_DIAMETER;
    }


    public void setModuleState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, new Rotation2d(getTurnHeading()));

        SmartDashboard.putNumber(m_config.DRIVE_MOTOR_ID + "_SPEED_TARGET", state.speedMetersPerSecond);
        SmartDashboard.putNumber(m_config.TURN_MOTOR_ID + "_TURN_TARGET", state.angle.getRotations());
        // CTRE
        m_velocityCommand.Velocity = state.speedMetersPerSecond / (Math.PI * m_config.WHEEL_DIAMETER);
        m_positionCommand.Position = state.angle.getRotations();

        m_driveCTRE.setControl(m_velocityCommand);
        m_turnCTRE.setControl(m_positionCommand);
    }
}
