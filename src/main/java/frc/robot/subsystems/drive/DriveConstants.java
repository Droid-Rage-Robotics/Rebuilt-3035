package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.Getter;

public class DriveConstants {
    public static class ModuleConstants {
        public static final Distance WHEEL_DIAMETER = Inches.of(4);
        // public static final double DRIVE_MOTOR_GEAR_RATIO = GearRatio.R3.getConversionFactor();
        // public static final double TURN_MOTOR_GEAR_RATIO = GearRatio.TURN.getConversionFactor();

        // public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER.in(Meters);
        // public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        public static final double READINGS_PER_REVOLUTION = 1;//4096

        //Used for the CANCoder
        public static final double TURN_ENCODER_ROT_2_RAD = 2 * Math.PI / READINGS_PER_REVOLUTION;
        public static final double TURN_ENCODER_ROT_2_RAD_SEC = TURN_ENCODER_ROT_2_RAD/60;

        /* Current Limits */
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 35;
        public static final double DRIVE_STATOR_CURRENT_LIMIT = 75;
        public static final double TURN_SUPPLY_CURRENT_LIMIT = 80;
    }


    public enum SwerveDriveConfig {
        TRACK_WIDTH(Units.inchesToMeters(28.5)),//Units.inchesToMeters(28.5)
        WHEEL_BASE(Units.inchesToMeters(28.5)),//Units.inchesToMeters(28.5)

        MAX_ACCELERATION_UNITS_PER_SECOND(10),
        MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND(10),

        MAX_SPEED_METERS_PER_SECOND(SwerveConfig.ATTAINABLE_MAX_SPEED.in(MetersPerSecond) / 4),
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND(SwerveConfig.ATTAINABLE_MAX_SPEED_ANG.in(RadiansPerSecond) / 10),
        MAX_ACCELERATION_METERS_PER_SECOND_SQUARED(1),
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED(1), // 1 / 8 of a full rotation per second per second),

        // Translational PID
        TRANSLATIONAL_KP(7),//3
        TRANSLATIONAL_KI(0),
        TRANSLATIONAL_KD(0),

        // Theta PID
        THETA_KP(5),
        THETA_KI(0),
        THETA_KD(0),

        // Turn PID for Swerve Pod
        TURN_KP(.4),//1

        // Drive Feedforward
        DRIVE_KS(0.13), // this value is multiplied by veloicty in meteres per second
        DRIVE_KV(2.7), //this value is the voltage that iwll be constantly applied
        // DRIVE_KA = 0.12,

        // Bevel Gears to the Left <-
        // BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0),
        // BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0),
        // FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0),
        // FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(0),


        BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-2.5065246074051375),
        BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-0.8314175870340175),
        FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-2.06934008285773),
        FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS(-1.1535535524900022),        

        DEFAULT_HEADING_OFFSET(0),
        ;
        
        public double value;
        private SwerveDriveConfig(double value) {
            this.value = value;
        }
        
        public double getValue() {
            return value;
        }
    }

    public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(7,0,0);

    public static final PIDConstants THETA_PID = new PIDConstants(5,0,0);

    public enum DriveOptions { 
        IS_FIELD_ORIENTED(true),
        IS_SQUARED_INPUTS(true),
        IS_POSE_UPDATED(true)
        ;
        private final boolean value;
        private DriveOptions(boolean value) {
            this.value = value;
        } 
        public boolean get(){
            return value;
        }
    }

    
}
