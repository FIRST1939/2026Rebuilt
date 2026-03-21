package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class ShiftUtil {
    
    public static class Shift {
        
        private final String m_name;
        private final boolean m_isActive;
        private final double m_secondsRemaining;

        public Shift (String name, boolean isActive, double secondsRemaining) {

            m_name = name;
            m_isActive = isActive;
            m_secondsRemaining = secondsRemaining;
        }

        public String getName() { return m_name; }
        public boolean isActive() { return m_isActive; }
        public double getSecondsRemaining() { return m_secondsRemaining; }
    }

    public static boolean activeFirst() {

        boolean redActiveFirst;
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) {

            redActiveFirst = false;
        } else {

            switch (gameData.charAt(0)) { // Who Won Auto
                case 'R' -> redActiveFirst = false;
                case 'B' -> redActiveFirst = true;
                default -> redActiveFirst = false;
            }
        }

        return Util.isRedAlliance() ? redActiveFirst : !redActiveFirst;
    }

    public static Shift getCurrentShift() {

        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous()) {

            return new Shift("Autonomous", true, matchTime);
        }

        boolean allianceActiveFirst = activeFirst();

        if (matchTime > 130) { return new Shift("Transition", true, matchTime - 130); } 
        if (matchTime > 105) { return new Shift("Shift 1", allianceActiveFirst, matchTime - 105); }
        if (matchTime > 80) { return new Shift("Shift 2", !allianceActiveFirst, matchTime - 80); }
        if (matchTime > 55) { return new Shift("Shift 3", allianceActiveFirst, matchTime - 55); }
        if (matchTime > 30) { return new Shift("Shift 4", !allianceActiveFirst, matchTime - 30); }

        return new Shift("Endgame", true, matchTime);
    }

    public static boolean fuelWillScore(double timeOfFlight) {

        Shift currentShift = getCurrentShift();
        double timeLeftToScore = currentShift.getSecondsRemaining() + 3;

        if (currentShift.getName().equals("Transition") && activeFirst()) {

            // The First Shift is Also Active
            timeLeftToScore += 25;
        }

        // 85% of Fuel Gets Processed in <= 1.75s
        // 92.5% of Fuel Gets Processed in <= 1.875s
        // 95% of Fuel Gets Processed in <= 2.00s
        double timeToProcess = 1.875;

        return timeOfFlight + timeToProcess <= timeLeftToScore;
    }
}
