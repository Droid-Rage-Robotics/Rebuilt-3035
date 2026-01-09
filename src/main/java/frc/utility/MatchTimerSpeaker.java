
package frc.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.sound.sampled.*;
import java.io.File;
import java.io.IOException;

public class MatchTimerSpeaker extends SubsystemBase {
    private double timeRemaining = 0; // time in seconds

    public MatchTimerSpeaker() {
    }

    @Override
    public void periodic() {
        timeRemaining = DriverStation.getMatchTime();

        if (timeRemaining != -1.0) {
                switch((int) timeRemaining) {
                    case 60: 
                        playSound("src/main/deploy/songs/60.wav");
                        break;
                    case 30:
                        playSound("src/main/deploy/songs/30.wav");
                        break;
                    case 15:
                        playSound("src/main/deploy/songs/15.wav");
                        break;
                    case 10:
                        playSound("src/main/deploy/songs/10.wav");
                        break;
                    case 5:
                        playSound("src/main/deploy/songs/5.wav");
                        break;
                    case 4:
                        playSound("src/main/deploy/songs/4.wav");
                        break;
                    case 3:
                        playSound("src/main/deploy/songs/3.wav");
                        break;
                    case 2:
                        playSound("src/main/deploy/songs/2.wav");
                        break;
                    case 1:
                        playSound("src/main/deploy/songs/1.wav");
                        break;
                }
        }

        //Double Check this does not interfere with other things
        try {
            Thread.sleep(1000); // Check every 100ms
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    private static void playSound(String filePath) {
        try {
            AudioInputStream audio = AudioSystem.getAudioInputStream(new File(filePath));
            Clip clip = AudioSystem.getClip();
            clip.open(audio);
            clip.start();
        } catch (UnsupportedAudioFileException | IOException | LineUnavailableException e) {
            e.printStackTrace();
        }
    }
}
