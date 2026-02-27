package frc.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubShift {
    
    public enum ShiftEnum {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED;
    }
    // private static Timer shiftTimer = new Timer();
    private static Alliance autoWinner;

    public static final double autoEndTime = 20.0;
    public static final double teleopDuration = 140.0;

    private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static boolean[] activeSchedule;
    private static final boolean[] looseSchedule = {true, true, false, true, false, true};
    private static final boolean[] winSchedule = {true, false, true, false, true, true};
    private static final ShiftEnum[] shiftsEnums = {ShiftEnum.TRANSITION, ShiftEnum.SHIFT1, 
        ShiftEnum.SHIFT2, ShiftEnum.SHIFT3, ShiftEnum.SHIFT4, ShiftEnum.ENDGAME};
    private double timeLeft = -1.0;

    public HubShift(){
        
    }
    
    public void updateShiftMessage() {
        timeLeft = DriverStation.getMatchTime();
        // Publish match time
        SmartDashboard.putNumber("Misc/Match Time", timeLeft);

        // Update from HubShiftUtil
        SmartDashboard.putNumber(
            "Misc/Shifts/Remaining Shift Time",getShiftRemainingTime());
        SmartDashboard.putBoolean("Misc/Shifts/Shift Active", 
            isShiftActive());
        SmartDashboard.putString(
            "Misc/Shifts/Game State", getShift().toString());
        SmartDashboard.putBoolean(
            "Misc/Shifts/Win Auto?",
            DriverStation.getAlliance().get() == autoWinner); //True = Green, False = Red
    }

    public void setWinner(){
        switch(DriverStation.getGameSpecificMessage()){
            case "B":
            autoWinner = Alliance.Blue; 
            case "R":
            autoWinner = Alliance.Red;
        }
        if(DriverStation.getAlliance().get() == autoWinner){
            activeSchedule = winSchedule;
        } else {
            activeSchedule = looseSchedule;
        }
    }
    public ShiftEnum getShift(){
        if(DriverStation.isAutonomousEnabled()){
            if(timeLeft > autoEndTime){
                return ShiftEnum.AUTO;
            }
        } else if(DriverStation.isTeleopEnabled()){
            for(int i = 0; i < shiftStartTimes.length; i++){
                if(timeLeft <= shiftEndTimes[i] && timeLeft > shiftStartTimes[i]){
                    return shiftsEnums[i];
                }
            }
        } else {
            return ShiftEnum.DISABLED;
        }
        return ShiftEnum.DISABLED;
    }
    public boolean isShiftActive(){
        if(DriverStation.isAutonomousEnabled()){
            if(timeLeft > autoEndTime){
                return true;
            }
        } else if(DriverStation.isTeleopEnabled()){
            for(int i = 0; i < shiftStartTimes.length; i++){
                if(timeLeft <= shiftEndTimes[i] && timeLeft > shiftStartTimes[i]){
                    return activeSchedule[i];
                }
            }
        } else {
            return false;
        }
        return false;
    }
    public double getShiftRemainingTime(){
        if(DriverStation.isAutonomousEnabled()){
            if(timeLeft > autoEndTime){
                return timeLeft - autoEndTime;
            }
        } else if(DriverStation.isTeleopEnabled()){
            for(int i = 0; i < shiftStartTimes.length; i++){
                if(timeLeft <= shiftEndTimes[i] && timeLeft > shiftStartTimes[i]){
                    return timeLeft - shiftStartTimes[i];
                }
            }
        } else {
            return -1.0;
        }
        return -1.0;
    }
}
