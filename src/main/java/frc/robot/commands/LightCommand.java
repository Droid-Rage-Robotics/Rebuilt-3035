package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Light;

public class LightCommand extends Command {
    public enum LightState {
        BATTERY_LOW, //Battery Blue
        PAUSE, //Decoration - Blue/Orange

        READY_TO_SCORE, // Score Fuel - Green
        READY_TO_SHOOT, // Hoard Fuel - Green
        NOT_READY, //Not Ready to Shoot - Red

        ALLIANCE_SHIFT, // Orange
        OPPONENT_SHIFT // White
    }

    private Light light;
    private LightState shootState = LightState.PAUSE;
    private Timer intakeTimer = new Timer();
    private Timer elementInTimer = new Timer();
    private boolean didWeWin = true;

    public LightCommand(Light light) {
        this.light = light;
    }

    @Override
    public void initialize() {
        intakeTimer.start();
        elementInTimer.start();
        if (DroidRageConstants.BatteryLow) {
            shootState = LightState.BATTERY_LOW; //ToDo: Is this different enough from the other light
            light.setAll(light.batteryBlue);
        } else {
            shootState = LightState.PAUSE;
            light.setAlternating(light.DRBlue, light.DROrange); //To Test
        }

    }

    @Override
    public void execute() {
        shootLights();
        shiftLights();
    }
    
    public void shootLights(){
        switch (shootState) {
            case READY_TO_SCORE: //Score Fuel
                light.setShootHalf(light.green);
                break;
            case READY_TO_SHOOT: //Hoard Fuel
                light.setShootHalf(light.green);
                break;
            case NOT_READY: // Hoard Fuel
                light.setShootHalf(light.red);
                break;
            default:
                break;
        }
    }

    public void setPeriod(){
        switch (DriverStation.getGameSpecificMessage()) {
            case "R": //Red won Auto
                didWeWin = DriverStation.getAlliance().get()== DriverStation.Alliance.Red;
                break;
            case "B": //Blue won Auto
                didWeWin = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
                break;
        }
    }

    public void shiftLights(){
        double matchTime = DriverStation.getMatchTime();
        if(didWeWin)
        if(matchTime<30){
            light.setShiftHalf(light.orange); //Endgame and Auto Shift
        } else if (matchTime<55&&matchTime>30){
            light.setShiftHalf(light.white); //Shift 4 
        } else if (matchTime < 55 && matchTime > 30) {
            light.setShiftHalf(light.orange); // Shift 3
        } else if (matchTime<55&&matchTime>30){
            light.setShiftHalf(light.white); //Shift 2
        } else if (matchTime<55&&matchTime>30){
            light.setShiftHalf(light.orange); //Shift 1 
        } 



        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            light.setShiftHalf(light.orange); //Opponent Shift
        } else {
            light.setShiftHalf(light.white); //Alliance Shift
        }
    }
    @Override
    public void end(boolean interrupted) {
        shootState = LightState.PAUSE;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    // public double getMatchTime() {// TODO:test
    //     return DriverStation.getMatchTime();
    // }
}
