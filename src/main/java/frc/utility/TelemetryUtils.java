package frc.utility;

import java.util.ArrayList;
import java.util.List;

public class TelemetryUtils {
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

    public interface Dashboard {
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

    public interface TelemetryUpdater {
        public void updateTelemetry();
    }

    private static final List<Dashboard> dashboardPublishers = new ArrayList<>();
    private static final List<TelemetryUpdater> telemetryUpdaters = new ArrayList<>();
    
    /**
     * Call this function to register the subsystem's {@code elasticInit()} method
     * to be run at robot startup
     * @param subsystem set to {@code this} while in a subsystem class
     */
    public static void registerDashboard(Dashboard subsystem) {
        dashboardPublishers.add(subsystem);
    }

    /**
     * Call this function to register a class's {@code updateTelemetry()} method
     * to be run periodically
     * 
     * @param value set to {@code this} while in a class
     */
    public static void registerTelemetry(TelemetryUpdater value) {
        telemetryUpdaters.add(value);
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
    }

    /**
     * Elastic configurations to run while the robot is disabled. Call this once
     * in {@link frc.robot.Robot#disabledPeriodic()}
     */
    public static void onDisabledPeriodic() {}

    /**
     * Configurations to run periodically. Call this once
     * in {@link frc.robot.Robot#robotPeriodic()}
     */
    public static void onRobotPeriodic() {
        for (TelemetryUpdater pub : telemetryUpdaters) {
            pub.updateTelemetry();
        }
    }
}