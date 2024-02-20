package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModule.SwerveModuleConfig;

public final class Constants {
    public static final class MyCANID {

        public static final int leftShooter = 14;
        public static final int rightShooter = 15;
        public static final int intake = 16;
    }

    public static final class SwerveDriveConstants {
        public static final double MAX_MODULE_SPEED_METERS_PER_SECOND = 4;

        public static final double MAX_ROBOT_LINEAR_SPEED_METERS_PER_SECOND = 4;
        public static final double MAX_ROBOT_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;

        public static final SwerveModuleConfig FL_MODULE_CONFIG = new SwerveModuleConfig();
        public static final SwerveModuleConfig FR_MODULE_CONFIG = new SwerveModuleConfig();
        public static final SwerveModuleConfig RL_MODULE_CONFIG = new SwerveModuleConfig();
        public static final SwerveModuleConfig RR_MODULE_CONFIG = new SwerveModuleConfig();
        public static final SwerveModuleConfig SR_MODULE_CONFIG = new SwerveModuleConfig();
        // Make sure to call this method to set your values before doing anything!
        public static void setupSwerveModuleConfigs() {
            // Front left module
            FL_MODULE_CONFIG.TURN_MOTOR_ID = 5;
            FL_MODULE_CONFIG.TURN_MOTOR_GEARING = 150/7.1;
            FL_MODULE_CONFIG.TURN_CONTROLLER_P = 10;
            FL_MODULE_CONFIG.TURN_CONTROLLER_D = 0;
            FL_MODULE_CONFIG.TURN_CONTROLLER_KS = 0;
            FL_MODULE_CONFIG.TURN_CONTROLLER_KV = 0;
            FL_MODULE_CONFIG.TURN_INVERSE = true;
            FL_MODULE_CONFIG.TURN_ENCODER_INVERSE = false;
            FL_MODULE_CONFIG.TURN_ENCODER_OFFSET = 0.6022;
            FL_MODULE_CONFIG.TURN_ENCODER_ID = 3;

            FL_MODULE_CONFIG.DRIVE_MOTOR_ID = 10;
            FL_MODULE_CONFIG.DRIVE_MOTOR_GEARING = 6.75;
            FL_MODULE_CONFIG.DRIVE_CONTROLLER_P = 1;
            FL_MODULE_CONFIG.DRIVE_CONTROLLER_D = 0;
            FL_MODULE_CONFIG.DRIVE_CONTROLLER_KS = 0;
            FL_MODULE_CONFIG.DRIVE_CONTROLLER_KV = 12 / (5.092 / Units.inchesToMeters(4) * Math.PI) * FL_MODULE_CONFIG.DRIVE_MOTOR_GEARING;
            FL_MODULE_CONFIG.DRIVE_INVERSE = false;

            FL_MODULE_CONFIG.WHEEL_DIAMETER = Units.inchesToMeters(4.0);

            FL_MODULE_CONFIG.MODULE_LOCATION = new Translation2d(0.2032, 0.2032);

            // Front right module
            FR_MODULE_CONFIG.TURN_MOTOR_ID = 2;
            FR_MODULE_CONFIG.TURN_MOTOR_GEARING = 150/7.1;
            FR_MODULE_CONFIG.TURN_CONTROLLER_P = 10;
            FR_MODULE_CONFIG.TURN_CONTROLLER_D = 0;
            FR_MODULE_CONFIG.TURN_CONTROLLER_KS = 0;
            FR_MODULE_CONFIG.TURN_CONTROLLER_KV = 0;
            FR_MODULE_CONFIG.TURN_INVERSE = true;
            FR_MODULE_CONFIG.TURN_ENCODER_INVERSE = false;
            FR_MODULE_CONFIG.TURN_ENCODER_OFFSET = 0.4671;
            FR_MODULE_CONFIG.TURN_ENCODER_ID = 2;

            FR_MODULE_CONFIG.DRIVE_MOTOR_ID = 12;
            FR_MODULE_CONFIG.DRIVE_MOTOR_GEARING = 6.75;
            FR_MODULE_CONFIG.DRIVE_CONTROLLER_P = 1;
            FR_MODULE_CONFIG.DRIVE_CONTROLLER_D = 0;
            FR_MODULE_CONFIG.DRIVE_CONTROLLER_KS = 0.1;
            FR_MODULE_CONFIG.DRIVE_CONTROLLER_KV = 12 / (5.092 / Units.inchesToMeters(4) * Math.PI) * FR_MODULE_CONFIG.DRIVE_MOTOR_GEARING;
            FR_MODULE_CONFIG.DRIVE_INVERSE = true;

            FR_MODULE_CONFIG.WHEEL_DIAMETER = Units.inchesToMeters(4.0);

            FR_MODULE_CONFIG.MODULE_LOCATION = new Translation2d(0.2032, -0.2032);

            // Rear left module
            RL_MODULE_CONFIG.TURN_MOTOR_ID = 7;
            RL_MODULE_CONFIG.TURN_MOTOR_GEARING = 150/7.1;
            RL_MODULE_CONFIG.TURN_CONTROLLER_P = 10;
            RL_MODULE_CONFIG.TURN_CONTROLLER_D = 0;
            RL_MODULE_CONFIG.TURN_CONTROLLER_KS = 0;
            RL_MODULE_CONFIG.TURN_CONTROLLER_KV = 0;
            RL_MODULE_CONFIG.TURN_INVERSE = true;
            RL_MODULE_CONFIG.TURN_ENCODER_INVERSE = false;
            RL_MODULE_CONFIG.TURN_ENCODER_OFFSET = 0.8792;
            RL_MODULE_CONFIG.TURN_ENCODER_ID = 0;

            RL_MODULE_CONFIG.DRIVE_MOTOR_ID = 1;
            RL_MODULE_CONFIG.DRIVE_MOTOR_GEARING = 6.75;
            RL_MODULE_CONFIG.DRIVE_CONTROLLER_P = 1;
            RL_MODULE_CONFIG.DRIVE_CONTROLLER_D = 0;
            RL_MODULE_CONFIG.DRIVE_CONTROLLER_KS = 0.1;
            RL_MODULE_CONFIG.DRIVE_CONTROLLER_KV = 12 / (5.092 / Units.inchesToMeters(4) * Math.PI) * RL_MODULE_CONFIG.DRIVE_MOTOR_GEARING;
            RL_MODULE_CONFIG.DRIVE_INVERSE = false;

            RL_MODULE_CONFIG.WHEEL_DIAMETER = Units.inchesToMeters(4.0);

            RL_MODULE_CONFIG.MODULE_LOCATION = new Translation2d(-0.2032, 0.2032);

            // Rear right module
            RR_MODULE_CONFIG.TURN_MOTOR_ID = 4;
            RR_MODULE_CONFIG.TURN_MOTOR_GEARING = 150/7.1;
            RR_MODULE_CONFIG.TURN_CONTROLLER_P = 10;
            RR_MODULE_CONFIG.TURN_CONTROLLER_D = 0;
            RR_MODULE_CONFIG.TURN_CONTROLLER_KS = 0;
            RR_MODULE_CONFIG.TURN_CONTROLLER_KV = 0;
            RR_MODULE_CONFIG.TURN_INVERSE = true;
            RR_MODULE_CONFIG.TURN_ENCODER_INVERSE = false;
            RR_MODULE_CONFIG.TURN_ENCODER_OFFSET = 0.1986;
            
            RR_MODULE_CONFIG.TURN_ENCODER_ID = 1;

            RR_MODULE_CONFIG.DRIVE_MOTOR_ID = 0;
            RR_MODULE_CONFIG.DRIVE_MOTOR_GEARING = 6.75;
            RR_MODULE_CONFIG.DRIVE_CONTROLLER_P = 1;
            RR_MODULE_CONFIG.DRIVE_CONTROLLER_D = 0;
            RR_MODULE_CONFIG.DRIVE_CONTROLLER_KS = 0.1;
            RR_MODULE_CONFIG.DRIVE_CONTROLLER_KV = 12 / (5.092 / Units.inchesToMeters(4) * Math.PI) * RR_MODULE_CONFIG.DRIVE_MOTOR_GEARING;
            RR_MODULE_CONFIG.DRIVE_INVERSE = true;

            RR_MODULE_CONFIG.WHEEL_DIAMETER = Units.inchesToMeters(4.0);

            RR_MODULE_CONFIG.MODULE_LOCATION = new Translation2d(-0.2032, -0.2032);
        }
    }
}
