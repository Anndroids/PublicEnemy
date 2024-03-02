package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleOpCommand extends Command {
    private final double DEADBAND=0.1;
    private final DoubleSupplier m_xAxis;
    private final DoubleSupplier m_yAxis;
    private final DoubleSupplier m_omega;

    private final DrivetrainSubsystem m_drive;

    public TeleOpCommand(final DoubleSupplier x, final DoubleSupplier y, final DoubleSupplier omega, final DrivetrainSubsystem drive) {
        m_xAxis = x;
        m_yAxis = y;
        m_omega = omega;
        m_drive = drive;

        addRequirements(m_drive);
    }



    @Override
    public void initialize() {
 
    }

    @Override
    public void execute() {
        m_drive.setChassisSpeeds(
            (Math.abs(m_xAxis.getAsDouble())<DEADBAND ? 0 : -m_xAxis.getAsDouble()) * Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND,
            (Math.abs(m_yAxis.getAsDouble())<DEADBAND ? 0 : -m_yAxis.getAsDouble()) * Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND,
            (Math.abs(m_omega.getAsDouble())<DEADBAND ? 0 : -m_omega.getAsDouble()) * Constants.SwerveDriveConstants.MAX_ROBOT_ANGULAR_SPEED_RADIANS_PER_SECOND
    
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}