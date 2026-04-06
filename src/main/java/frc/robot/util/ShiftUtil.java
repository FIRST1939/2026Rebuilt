package frc.robot.util;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

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

    public static Optional<Boolean> activeFirst() {

        Optional<Boolean> redActiveFirst;
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty() || DriverStation.isAutonomous() || DriverStation.isDisabled()) {

            redActiveFirst = Optional.empty();
        } else {

            switch (gameData.charAt(0)) { // Who Won Auto
                case 'R' -> redActiveFirst = Optional.of(false);
                case 'B' -> redActiveFirst = Optional.of(true);
                default -> redActiveFirst = Optional.empty();
            }
        }

        if (redActiveFirst.isEmpty()) { return redActiveFirst; }
        return Util.isRedAlliance() ? redActiveFirst : Optional.of(!redActiveFirst.get());
    }

    public static Shift getCurrentShift() {

        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous()) {

            return new Shift("Autonomous", true, matchTime);
        }

        Optional<Boolean> optionalActiveFirst = activeFirst();
        boolean allianceActiveFirst = activeFirst().isEmpty() ? Util.isRedAlliance() : optionalActiveFirst.get();

        if (matchTime > 130) { return new Shift("Transition", true, matchTime - 130); } 
        if (matchTime > 105) { return new Shift("Shift 1", allianceActiveFirst, matchTime - 105); }
        if (matchTime > 80) { return new Shift("Shift 2", !allianceActiveFirst, matchTime - 80); }
        if (matchTime > 55) { return new Shift("Shift 3", allianceActiveFirst, matchTime - 55); }
        if (matchTime > 30) { return new Shift("Shift 4", !allianceActiveFirst, matchTime - 30); }

        return new Shift("Endgame", true, matchTime);
    }

    public static boolean fuelWillScore(double timeOfFlight) {

        if (DriverStation.getMatchTime() < 0) { return true; }

        Optional<Boolean> optionalActiveFirst = activeFirst();
        boolean allianceActiveFirst = activeFirst().isEmpty() ? Util.isRedAlliance() : optionalActiveFirst.get();

        Shift currentShift = getCurrentShift();
        double timeLeftInShift = currentShift.getSecondsRemaining();
        if (currentShift.isActive()) { timeLeftInShift += 3; }

        if (currentShift.getName().equals("Transition") && allianceActiveFirst) {

            // The First Shift is Also Active
            timeLeftInShift += 25;
        }

        if (currentShift.getName().equals("Shift 4") && !allianceActiveFirst) {

            // Endgame is Also Active
            timeLeftInShift += 30;
        }

        // 85% of Fuel Gets Processed in <= 1.75s
        // 92.5% of Fuel Gets Processed in <= 1.875s
        // 95% of Fuel Gets Processed in <= 2.00s
        double timeToProcess = 1.875;

        Logger.recordOutput("Time Left in Shift", timeLeftInShift);
        Logger.recordOutput("Total Time to Score", timeOfFlight + timeToProcess);

        if (currentShift.isActive()) {

            return timeOfFlight + timeToProcess <= timeLeftInShift;
        }

        return timeOfFlight + timeToProcess >= timeLeftInShift;
    }
}
