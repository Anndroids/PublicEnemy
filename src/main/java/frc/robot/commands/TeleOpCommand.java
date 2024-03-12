package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleOpCommand extends Command {
    private final double DEADBAND=0.1;
    private final DoubleSupplier m_xAxis;
    private final DoubleSupplier m_yAxis;
    private final DoubleSupplier m_omega;

    private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(8);
    private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(8);
    private final SlewRateLimiter omegaSlewRateLimiter = new SlewRateLimiter(Math.PI * 4);


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
        xSlewRateLimiter.reset(0);
        ySlewRateLimiter.reset(0);
        omegaSlewRateLimiter.reset(0);
    }

    @Override
    public void execute() {
        m_drive.setChassisSpeeds(
            xSlewRateLimiter.calculate(    
                (Math.abs(m_xAxis.getAsDouble())<DEADBAND ? 0 : -m_xAxis.getAsDouble()) * Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND),
            ySlewRateLimiter.calculate(
                (Math.abs(m_yAxis.getAsDouble())<DEADBAND ? 0 : -m_yAxis.getAsDouble()) * Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND),
            omegaSlewRateLimiter.calculate(
                (Math.abs(m_omega.getAsDouble())<DEADBAND ? 0 : -m_omega.getAsDouble()) * Constants.SwerveDriveConstants.MAX_ROBOT_ANGULAR_SPEED_RADIANS_PER_SECOND)
    
        );
    }

     // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setChassisSpeeds(0,0,0);
  }


    @Override
    public boolean isFinished() {
        return false;
    }
    
}