package frc.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DroidRageConstants;

public class DashboardUtils {
    /**
     * Determines whether writers placed in {@code practiceWriters()}
     * are enabled or disabled
     */
    public enum MatchValue {
        /**
         * Writers placed in {@code practiceWriters()} will be disabled
         * to prevent loop overruns at a competition.
         */
        COMPETITION,

        /**
         * Writers placed in {@code practiceWriters()} will be enabled
         * for access to testing data in Elastic.
         */
        PRACTICE
    }

    public static class Config {
        /**
         * Default is {@code MatchValue.PRACTICE}
         */
        public static MatchValue Match = MatchValue.PRACTICE;
    }

    public interface Dashboard{
        /**
         * Place all elastic configs in here to be run at robot startup
         */
        public void elasticInit();

        /**
         * Place all writers that are not neccesary during a match here 
         * to only be used during practice to prevent loop overruns
         */
        public void practiceWriters();

        
        /**
         * Place any logic for alerts here to be run periodically
         */
        public void alerts();
    }

    public interface Periodic {
        public void periodic();
    }

    private static final List<Dashboard> dashboardPublishers = new ArrayList<>();
    private static final List<Periodic> periodicPublishers = new ArrayList<>();

    private static final Alert batteryAlert = new Alert("Battery Voltage", AlertType.kError);
    private static final Elastic.Notification notification = new Elastic.Notification();
    private static final PowerDistribution powerDistribution = new PowerDistribution();
    
    /**
     * Call this function to register the subsystem's {@code elasticInit()} method
     * to be run at robot startup
     * @param subsystem set to {@code this} while in a subsystem class
     */
    public static void registerDashboard(Dashboard subsystem) {
        dashboardPublishers.add(subsystem);
    }
    
    /**
     * Call this function to register a class's {@code periodic()} method
     * to be run at robot startup
     * 
     * <p>DO NOT USE THIS IN A SUBSYSTEM!!!
     * @param value set to {@code this} while in a class
     */
    public static void registerPeriodic(Periodic value) {
        periodicPublishers.add(value);
    }
    
    /**
     * Elastic configurations to run when the robot initializes. Call this once
     * in {@code Robot.robotInit()}
     */
    public static void onRobotInit() {
        for (Dashboard pub : dashboardPublishers) {
            pub.elasticInit();

            if(Config.Match==MatchValue.PRACTICE) {
                pub.practiceWriters();
            }
        }

        if (DroidRageConstants.BatteryLow.get()) {
            Elastic.sendNotification(notification
                .withLevel(Elastic.NotificationLevel.ERROR)
                .withTitle("Battery")
                .withDescription("Battery Low!")
                .withDisplaySeconds(10.0));
        }

        SmartDashboard.putData("Distribution", powerDistribution);
    }

    /**
     * Elastic configurations to run while the robot is disabled. Call this once
     * in {@code Robot.disabledPeriodic()}
     */
    public static void onDisabledPeriodic() {
        if (DroidRageConstants.BatteryLow.get()) {
            batteryAlert.set(true);
            batteryAlert.setText("Battery Voltage Low");
        } else {
            batteryAlert.set(false);
        }
    }

    /**
     * Configurations to run periodically. Call this once
     * in {@code Robot.robotPeriodic()}
     */
    public static void onRobotPeriodic() {
        for (Periodic pub : periodicPublishers) {
            pub.periodic();
        }
    }
}