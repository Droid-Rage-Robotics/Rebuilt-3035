package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.intake.Intake.IntakeValue;
import frc.utility.devices.motor.MotorConstants;
import frc.utility.devices.motor.MotorConstants.Direction;
import frc.utility.template.FlywheelTemplate;
import frc.utility.template.SubsystemConstants;
import frc.utility.template.SubsystemConstants.EncoderType;
import lombok.Getter;

public class Indexer extends FlywheelTemplate{
    public enum IndexerValue {
        INTAKE(10), //100
        OUTTAKE(5),//-25
        STOP(0),
        HOLD(0);

        @Getter private final AngularVelocity indexerValue;

        private IndexerValue(double indexerValue) {
            this.indexerValue = RotationsPerSecond.of(indexerValue);
        }
    }
    
    private static final SubsystemConstants constants = new SubsystemConstants()
        .withConversionFactor(1.0/9.0)
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
    private Timer stallTimer = new Timer();
    // private double intakeTime = 0;
    private boolean isStalling = false;
    private boolean isReversing = false;

    public Indexer(boolean isEnabled) {
        super(isEnabled,
            new PIDController(0.014761, 0, 0), //0.032889
            new SimpleMotorFeedforward(0.14827, 1.0665, 0.014828), //0.34224, 0.37116, 0.0095347
            constants, 
            motor);
    }
        @Override
    public void periodic(){
        super.periodic();

    if (Math.abs(getCurrent().in(Amps)) >=50){
        if (!isStalling){
            isStalling = true;
            stallTimer.reset();
            stallTimer.start();
            System.out.println("INDEXER STALL");
        }
        System.out.println(stallTimer.get());

        if (stallTimer.get()> 0.2 && stallTimer.get()<1.5){
            System.out.println("REVERSING");
            isReversing = true;
            setTargetVelocity(IndexerValue.OUTTAKE.indexerValue);
            // stallTimer.reset();
        } 
        // else{
        //     System.out.println("REVERTING BACK");
        //     isReversing = false;
        //     isStalling = false;
        //     setTargetVelocity(IndexerValue.INTAKE.indexerValue);
        // }

        // if (isReversing && stallTimer.get()>1){
        //     System.out.println("REVERTING BACK");
        //     isReversing = false;
        //     isStalling = false;
        //     setTargetVelocity(IndexerValue.INTAKE.indexerValue);
        // } 
    } else {
        isStalling = false;
        isReversing = false;
        stallTimer.reset();

    }

// }
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
}
