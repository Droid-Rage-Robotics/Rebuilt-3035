package frc.utility.template;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.utility.motor.MotorBase;

//Works
public class SetPower {
    private final MotorBase[] motors;
    // private final ShuffleboardValue<Double> powerWriter;
    private final int mainNum;

    public SetPower(
        MotorBase[] motors,
        String name,
        int mainNum
    ){
        this.motors=motors;
        this.mainNum=mainNum;
        // powerWriter = ShuffleboardValue
        //     .create(0.0, name+"/Power", name)
        //     .build();
    }

    public Command setTargetPowerCommand(double power){
        return new InstantCommand(()->setTargetPower(power));
    }

    /*
     * Use this for initialization
     */
    public void setTargetPower(double power) {
        // powerWriter.set(power);
        for (MotorBase motor: motors) {
            motor.setPower(power);
        }
    }
   
    public MotorBase getMotor() {
        return motors[mainNum];
    }

    public MotorBase[] getAllMotor() {
        return motors;
    } 
}
