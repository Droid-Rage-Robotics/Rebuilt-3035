package frc.utility;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class AudioNotifier {
    private static final StringPublisher audioPub =
        NetworkTableInstance.getDefault()
            .getTable("Audio")
            .getStringTopic("Play")
            .publish();

    public static void play(String soundName) {
        audioPub.set(soundName);
    }
}