package frc.robot;

import static edu.wpi.first.units.Units.*;


import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class DroidRageConstants {
    public enum Alignment {
        RIGHT,
        LEFT,
        MIDDLE
    }
    
    public static Alignment alignmentMode = Alignment.LEFT;

    private static final AtomicReference<String> alignmentWriter = new AtomicReference<>(alignmentMode.toString());

    public static final AngularVelocity KRAKEN_X60_MAX_VELOCITY = RPM.of(6000);
    public static final AngularVelocity KRAKEN_X44_MAX_VELOCITY = RPM.of(7750);

    public static final double LOOP_PERIOD_SECS = 0.02;


    public static void setAlignment(Alignment alignment){
        alignmentMode = alignment;
        alignmentWriter.set(alignmentMode.toString());
    }


    public static final Sendable robotMisc = new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addStringProperty("Vision Alignment", alignmentWriter::get, null);
        };
    };

    
    
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
    public static final String leftLL = "limelight-left";
    public static final String rightLL = "limelight-right";
    public static final String middleLL = "limelight-middle";
    
    public interface MutableSupplier<T> {
        T get();
        void set(T value);
    }

    public static Supplier<Boolean> BatteryLow = () -> RobotController.getBatteryVoltage()<12.5;
    
    public static boolean didWeWin = true; //Win Auto?

    public static class FieldConstants {
        public static final Distance FUNNEL_RADIUS = Inches.of(24);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

        public static final Distance NEUTRAL_ZONE_START = Inches.of(0);
        public static final Distance NEUTRAL_ZONE_END = Inches.of(0);

        public static final Translation3d HUB_BLUE = new Translation3d(4.625, 4.025, 2.05);
        public static final Translation3d HUB_RED = new Translation3d(); // TODO: set

    }
    

    public enum SimState {
        REAL,
        SIM
    }

    public static SimState simState = SimState.REAL;

    public static Alliance alliance = DriverStation.Alliance.Blue; //Default to Blue
}
