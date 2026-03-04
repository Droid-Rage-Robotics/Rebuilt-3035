package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import lombok.Getter;

public class Indexer extends FlywheelTemplate{
    public enum IndexerValue {
        INTAKE(100),
        OUTTAKE(-25),
        STOP(0),
        HOLD(0);

        @Getter private final AngularVelocity indexerValue;

        private IndexerValue(double indexerValue) {
            this.indexerValue = RotationsPerSecond.of(indexerValue);
        }
    }
    
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1.0/3.0)
        .withEncoderType(EncoderType.INTEGRATED)
        .withMinVelocity(RotationsPerSecond.of(-150))
        .withMaxVelocity(RotationsPerSecond.of(150))
        .withName("Indexer")
        .withOffset(0)
        .withMainNum(0);
    
    private static final MotorConstants motor = new MotorConstants() 
        .withDeviceId(14)
        .withCANBus(DroidRageConstants.driveCanBus)
        .withDirection(Direction.Forward)
        .withIdleMode(NeutralModeValue.Coast)
        .withStatorCurrentLimit(80)//100
        .withSupplyCurrentLimit(80);//100
        //SUPERNERDS have 40 stator
    // private Timer stallTimer = new Timer();
    // private double intakeTime = 0;

    public Indexer(boolean isEnabled) {
        super(isEnabled,
            new PIDController(0.5, 0, 0), //0.032889
            new SimpleMotorFeedforward(0.34224, 0.37116, 0.01), //0.34224, 0.37116, 0.0095347
            constants, 
            motor);
    }

    // @Override
    // public void periodic() {
    //     super.periodic();

    //     if (Math.abs(getCurrent().in(Amps)) >= 50) {
    //         // setTargetVelocity(-getTargetVelocity());
    //         System.out.println("INDEXER STALL");
    //         // System.out.println(intakeTime);//4.7
    //         stallTimer.reset();
    //         // stallTimer
    //         System.out.println(stallTimer.get());//4

    //         if (stallTimer.get()>0.2) {
    //             System.out.println("REVERSING");
    //             setTargetVelocity(RotationsPerSecond.of(-getTargetVelocity()));
    //             stallTimer.restart();
    //             if (stallTimer.get()>0.5) {
    //                 System.out.println("REVERTING BACK");
    //                 setTargetVelocity(RotationsPerSecond.of(-getTargetVelocity()));
    //                 // intakeTime = 0;
    //             }
    //         }
    //     }
    // }

    // @Override
    // public Command setTargetVelocityCommand(AngularVelocity target) {
    //     if(target==IndexerValue.INTAKE.getIndexerValue()){
    //         intakeTime=stallTimer.get();
            
    //     }
    //     return super.setTargetVelocityCommand(target);
    // }
}
