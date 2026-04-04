package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class DroidRageConstants {

    public static final AngularVelocity KRAKEN_X60_MAX_VELOCITY = RPM.of(6000);
    public static final AngularVelocity KRAKEN_X44_MAX_VELOCITY = RPM.of(7750);

    public static final double LOOP_PERIOD_SECS = 0.02;
    
    public static class Gamepad {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DRIVER_STICK_DEADZONE = 0.025;
        public static final double OPERATOR_STICK_DEADZONE = 0.2;
    }

    public static double LOOP_TYPE_SECONDS = 0.02;

    public static double squareInput(double value) {
        return value * Math.abs(value);
    }

    public static double applyDeadBand(double value) {
        if (Math.abs(value) < DroidRageConstants.Gamepad.OPERATOR_STICK_DEADZONE) value = 0;
        return value;
    }

    public static boolean isWithinDeadzone(double stick) {
        return Math.abs(stick) < DroidRageConstants.Gamepad.OPERATOR_STICK_DEADZONE;
    }

    public class ControllerUtils {
        public static double getRightStickDeg(CommandXboxController controller) {
            double x = controller.getRightX();
            double y = controller.getRightY();

            // atan2 gives the angle of the vector (y, x)
            double angleRadians = Math.atan2(-y, -x);
            double angleDegrees = Math.toDegrees(angleRadians)-90;

            // Normalize to [0, 360)
            if (angleDegrees < 0) {
                angleDegrees += 360;
            }

            return angleDegrees;            
        }

        public static Rotation2d getRightStickRotation2d(CommandXboxController controller) {
            var rot = new Rotation2d(controller.getRightX(), controller.getRightY());
            rot = rot.plus(Rotation2d.fromDegrees(90));
            return rot;
        }

        public static double getLeftStickDeg(CommandXboxController controller) {
            double x = controller.getLeftX();
            double y = controller.getLeftY();

            // atan2 gives the angle of the vector (y, x)
            double angleRadians = Math.atan2(-y, -x);
            double angleDegrees = Math.toDegrees(angleRadians)-90;

            // Normalize to [0, 360)
            if (angleDegrees < 0) {
            angleDegrees += 360;
            }
            
            return angleDegrees;
        }
    }



    public static final CANBus driveCanBus = new CANBus("drive");
    public static final CANBus rioCanBus = new CANBus();
    public static final String leftLL = "limelight-left"; // 10.30.35.12
    public static final String rightLL = "limelight-right"; // 10.30.35.11
    public static final String middleLL = "limelight-middle";
    
    public interface MutableSupplier<T> {
        T get();
        void set(T value);
    }


    public static Supplier<Boolean> BatteryLow = () -> RobotController.getBatteryVoltage()<12.5;
    
    public static boolean didWeWin = true; //Win Auto?
    public static boolean isShooterManual = false; //True - Manual; False - Tracking
    public static final boolean isVisionEnabled = true;
    // public static setIsShooterManual(boolean isManual){
    //     isShooterManual = isManual;
    // }

    public static class FieldConstants {
        // public static final Distance FUNNEL_RADIUS = Inches.of(24);
        // public static final Distance FUNNEL_HEIGHT = Inches.of(69);

        public static final Translation2d HUB_BLUE = new Translation2d(4.625, 4.025);
        public static final Translation2d HUB_RED = new Translation2d(11.95,4.025);
        public static final Translation2d ALLIANCE_BLUE = new Translation2d(.5, 0);
        public static final Translation2d ALLIANCE_RED = new Translation2d(15.5,0);
    }

    public static Alliance alliance = DriverStation.Alliance.Blue; //Default to Blue
}
