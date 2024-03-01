package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.util.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveModule[] m_modules;
    public Pigeon2 gyro;
    private ChassisSpeeds m_speeds = new ChassisSpeeds();
    SwerveModuleState s = new SwerveModuleState();

    public DrivetrainSubsystem() {
        gyro = new Pigeon2(0);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.reset();

        m_modules = new SwerveModule[] { 
            new SwerveModule(Constants.SwerveDriveConstants.FL_MODULE_CONFIG),
            new SwerveModule(Constants.SwerveDriveConstants.FR_MODULE_CONFIG),
            new SwerveModule(Constants.SwerveDriveConstants.RL_MODULE_CONFIG),
            new SwerveModule(Constants.SwerveDriveConstants.RR_MODULE_CONFIG)
        };
        m_kinematics = new SwerveDriveKinematics(m_modules[0].m_location, m_modules[1].m_location, m_modules[2].m_location, m_modules[3].m_location);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("X", m_speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Y", m_speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Omega", m_speeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = m_kinematics. toSwerveModuleStates(m_speeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setModuleState(states[i]);
            m_modules[i].loop();
        }
    }

    public void setChassisSpeeds(double x, double y, double omega) {
        m_speeds.vxMetersPerSecond = x;
        m_speeds.vyMetersPerSecond = y;
        m_speeds.omegaRadiansPerSecond = omega;

        m_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, gyro.getRotation2d());
    }

public void resetGyroHeading() {
    gyro.reset();
}

    //public void setChassisSpeeds(double d, double e, double f, double g, double h) {
        // TODO Auto-generated method stub
       // throw new UnsupportedOperationException("Unimplemented method 'setChassisSpeeds'");
    //}
}
