package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants.Control;
import frc.utility.motor.MotorBase;
import frc.utility.motor.TalonEx;
import frc.utility.motor.MotorBase.Direction;
import frc.utility.motor.MotorBase.ZeroPowerMode;
import frc.utility.template.ElevatorTemplate;

public class Elevator extends ElevatorTemplate{
    public static class Constants {
        private static final double GEAR_RATIO = 12.0; // motor : sprocket
        private static final double SPROCKET_TEETH = 24;
        private static final double CHAIN_PITCH_METERS = Units.inchesToMeters(0.25);


        // Derived sprocket pitch diameter
        private static final double SPROCKET_DIAMETER_METERS =
            (CHAIN_PITCH_METERS * SPROCKET_TEETH) / Math.PI;

        // Circumference for one sprocket rev
        private static final double SPROCKET_CIRCUMFERENCE_METERS =
            Math.PI * SPROCKET_DIAMETER_METERS;

        public static final double MOTOR_ROT_2_METER = SPROCKET_CIRCUMFERENCE_METERS / GEAR_RATIO;

        public static final double MIN_HEIGHT = 0;
        public static final double MAX_HEIGHT = 50.5 * Constants.MOTOR_ROT_2_METER;   //40


    }

    public enum ElevatorValue {
        START(0),
        GROUND(0),
        INTAKE_HPS(0),
        CLIMB(0),
        
        L1(0),//5
        L2(7.2),//8
        L3(22),//29

        L4(50.5),
  
        ALGAE_LOW(24.1),// 18
        ALGAE_HIGH(40),//34.5
        BARGE(50.5),
        PROCESSOR(5)//13
        ;

        private final double height;

        private ElevatorValue(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height * Constants.MOTOR_ROT_2_METER; // convert rotations to meters
        }
    }

    public double resetPos = 7 * Constants.MOTOR_ROT_2_METER;

    private static TalonEx motorRight = TalonEx.create(15)
        .withDirection(Direction.Reversed)
        .withIdleMode(ZeroPowerMode.Coast)
        .withConversionFactor(1)
        .withSubsystem(null)
        .withIsEnabled(true)
        .withSupplyCurrentLimit(50);

    private static TalonEx motorLeft = TalonEx.create(14)
        .withDirection(Direction.Forward)
        .withIdleMode(ZeroPowerMode.Coast)
        .withConversionFactor(1)
        .withSubsystem(null)
        .withIsEnabled(true)
        .withSupplyCurrentLimit(50);
    
    public Elevator(boolean isEnabled) {
        super(
        new MotorBase[]{motorRight, motorLeft},
        80, 0, 0,         
        new ElevatorFeedforward(0.0374602, 0, 0.170813, 0), // correct as of 12/1/2025
        new TrapezoidProfile.Constraints(1.2, 1), // meters per sec
        // new ElevatorFeedforward(0.1, 0.18, 0.1868, 0),
        Constants.MAX_HEIGHT, Constants.MIN_HEIGHT, Constants.MOTOR_ROT_2_METER, 
        Control.TRAPEZOID_PROFILE, "Elevator", 0, isEnabled);
    }

    public Command setTargetPositionCommand(ElevatorValue target) {
        return setTargetPositionCommand(target.getHeight());
    }
}