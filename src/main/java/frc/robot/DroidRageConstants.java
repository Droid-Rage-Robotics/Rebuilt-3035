package frc.robot;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SuppliedCommand;

public final class DroidRageConstants {
    public enum Alignment {
        RIGHT,
        LEFT,
        MIDDLE
    }
    
    public static Alignment alignmentMode = Alignment.LEFT;

    private static final AtomicReference<String> alignmentWriter = new AtomicReference<>(alignmentMode.toString());

    public static void setAlignment(Alignment alignment){
        alignmentMode = alignment;
        alignmentWriter.set(alignmentMode.toString());
    }

    private static final AtomicReference<String> elementWriter = new AtomicReference<>(Element.NONE.toString());
    //All possible elements
        public enum Element{
        ALGAE,
        CORAL,
        NONE
    }

    public static final Sendable robotMisc = new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addStringProperty("Vision Alignment", alignmentWriter::get, null);
            builder.addStringProperty("Element", elementWriter::get, null);
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

    public static final CANBus driveCanBus = new CANBus("drive");
    public static final CANBus rioCanBus = new CANBus();
    public static final String leftLimelight = "limelight-left";
    public static final String rightLimelight = "limelight-right";

    public enum Control{
        PID,
        FEEDFORWARD,
        TRAPEZOID_PROFILE,
        SYS_ID
    }
    
    public interface MutableSupplier<T> {
        T get();
        void set(T value);
    }

    public static boolean BatteryLow = RobotController.getBatteryVoltage()<12.5;
    //TODO: Do you get battery Voltage?
    
    public static boolean didWeWin = true; //Win Auto?
}
