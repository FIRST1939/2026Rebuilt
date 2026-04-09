package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class MatchClock {
    
    private static Timer m_matchTimer = new Timer();
    private static double m_periodDuration;

    private static double m_timeOffset;
    private static double m_lastFMSTime;

    public static void startAutonomousClock() {

        m_matchTimer.restart();
        m_periodDuration = 20.0;
        m_timeOffset = 0.0;
    }

    public static void startTeleopClock() {

        m_matchTimer.restart();
        m_periodDuration = 140.0;
        m_timeOffset = 0.0;
    }

    public static void stopClock() {

        m_matchTimer.stop();
    }

    public static double getMatchTime() {

        return m_periodDuration - m_matchTimer.get() + m_timeOffset;
    }

    public static void syncFMS() {

        double fmsTime = Math.ceil(DriverStation.getMatchTime());

        if (fmsTime != m_lastFMSTime) {

            m_timeOffset = fmsTime - getMatchTime();
            m_lastFMSTime = fmsTime;
        }
    }
}
