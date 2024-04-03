package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.util.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveModule[] m_modules;
    public Pigeon2 gyro;
    private ChassisSpeeds m_speeds = new ChassisSpeeds();
    SwerveModuleState s = new SwerveModuleState();
    private Double GEAR_RATIO = 6.75;
    private Double WHEEL_DIAMETER = 4.0;

     private SwerveDriveOdometry odometry;

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
    
        odometry = new SwerveDriveOdometry(m_kinematics, gyro.getRotation2d(), getModulePositions());

        AutoBuilder.configureHolonomic(odometry::getPoseMeters,
            this::resetPose,
            this::getRobotChassisSpeeds,
            (speeds) -> m_speeds = speeds, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(5, 0, 0),
                new PIDConstants(4, 0, 0),
                SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND,
                SwerveDriveConstants.FL_MODULE_CONFIG.MODULE_LOCATION.getNorm(),
                new ReplanningConfig(),
                0.004),
            () -> DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false),
            this);
    
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("X", m_speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Y", m_speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Omega", m_speeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = m_kinematics. toSwerveModuleStates(m_speeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setModuleState(states[i]);
            m_modules[i].loop();
        }

        odometry.update(gyro.getRotation2d(), getModulePositions());
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

public void setGyroHeading(double setpoint){
    gyro.setYaw(setpoint);
}

public double getHeading(){
    return gyro.getAngle();
}

public void resetPose(Pose2d newPose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), newPose);
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[m_modules.length];
        for (int i = 0; i < modulePositions.length; i++) {
            SwerveModule module = m_modules[i];
            modulePositions[i] = new SwerveModulePosition(module.getDrivePosition(), Rotation2d.fromRotations(module.getTurnHeading()));
        }
        return modulePositions;
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_modules.length];
        for (int i = 0; i < moduleStates.length; i++) {
            SwerveModule module = m_modules[i];
            moduleStates[i] = new SwerveModuleState(module.getDriveVelocity(), Rotation2d.fromRotations(module.getTurnHeading()));
        }
        return moduleStates;
    }



}
