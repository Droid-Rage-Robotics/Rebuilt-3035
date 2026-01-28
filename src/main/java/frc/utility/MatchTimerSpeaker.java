package frc.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.sound.sampled.*;
import java.io.File;
import java.io.IOException;

public class MatchTimerSpeaker extends SubsystemBase {
    private double timeRemaining = 0; // time in seconds

    private int lastPlayedSecond = -1;

    @Override
    public void periodic() {
        timeRemaining = DriverStation.getMatchTime();
        if (timeRemaining != -1.0) {
            int currentSecond = (int) timeRemaining;
            
            // Only play if we haven't already played for this second
            if (currentSecond != lastPlayedSecond) {
                switch(currentSecond) {
                    case 60: 
                        playSound("songs/60.wav");
                        break;
                    case 30:
                        playSound("songs/30.wav");
                        break;
                    case 15:
                        playSound("songs/15.wav");
                        break;
                    case 10:
                        playSound("songs/10.wav");
                        break;
                    case 5:
                        playSound("songs/5.wav");
                        break;
                    case 4:
                        playSound("songs/4.wav");
                        break;
                    case 3:
                        playSound("songs/3.wav");
                        break;
                    case 2:
                        playSound("songs/2.wav");
                        break;
                    case 1:
                        playSound("songs/1.wav");
                        break;
                }
                lastPlayedSecond = currentSecond;
            }
        }
    }
    
    private static void playSound(String filePath) {
        try {
            AudioInputStream audio = AudioSystem.getAudioInputStream(new File(Filesystem.getDeployDirectory(), filePath));
            Clip clip = AudioSystem.getClip();
            clip.open(audio);
            clip.start();
        } catch (UnsupportedAudioFileException | IOException | LineUnavailableException e) {
            e.printStackTrace();
        }
    }
}
