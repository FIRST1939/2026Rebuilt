package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.feeder.Feeder;


public class ShotLogger {

    private static final double FEEDER_ACTIVE_VELOCITY_THRESHOLD = 100.0; 
    private static final double SHOT_CURRENT_THRESHOLD = 15.0;          // Amps 
    private static final double SHOT_CURRENT_COOLDOWN = 0.1;            // Seconds 

    private final Feeder m_feeder;

    // State tracking
    private boolean m_feederActive = false;
    private int m_shotCount = 0;
    private int m_setNumber = 0;
    private double m_setStartTime = 0.0;
    private boolean m_shotInProgress = false;
    private double m_cooldownTimer = 0.0;

    // Lifetime totals
    private int m_totalShots = 0;
    private double m_totalShootingTime = 0.0;
    private double m_averageShotsPerSecond = 0.0;

    public ShotLogger(Feeder feeder) {

        m_feeder = feeder;
    }

    
    public void update() {

        double velocity = Math.abs(m_feeder.getFeederVelocity());
        double current = m_feeder.getFeederCurrent();
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        boolean feederRunning = velocity > FEEDER_ACTIVE_VELOCITY_THRESHOLD;

        //Start
        if (feederRunning && !m_feederActive) {

            m_feederActive = true;
            m_shotCount = 0;
            m_setStartTime = now;
            m_shotInProgress = false;
            m_cooldownTimer = 0.0;
        }

        //Feeder Running
        if (m_feederActive && feederRunning) {

            if (m_cooldownTimer > 0.0) {

                m_cooldownTimer -= 0.02;
            } else if (current > SHOT_CURRENT_THRESHOLD && !m_shotInProgress) {

                // Rising edge of a current spike
                m_shotInProgress = true;
                m_shotCount++;
                m_totalShots++;
                m_cooldownTimer = SHOT_CURRENT_COOLDOWN;
            } else if (current <= SHOT_CURRENT_THRESHOLD) {

                m_shotInProgress = false;
            }
        }

        //Feeder Stopped
        if (!feederRunning && m_feederActive) {

            m_feederActive = false;

            if (m_shotCount > 0) {

                m_setNumber++;
                double duration = now - m_setStartTime;

                String prefix = "ShotLogger/Set " + m_setNumber;
                Logger.recordOutput(prefix + "/Shots", m_shotCount);
                Logger.recordOutput(prefix + "/Duration (s)", duration);
                Logger.recordOutput(prefix + "/Shots Per Second", duration > 0 ? m_shotCount / duration : 0.0);

                Logger.recordOutput("ShotLogger/Last Set Number", m_setNumber);
                Logger.recordOutput("ShotLogger/Last Set Shots", m_shotCount);
                Logger.recordOutput("ShotLogger/Last Set Duration (s)", duration);
                Logger.recordOutput("ShotLogger/Last Set Shots Per Second", duration > 0 ? m_shotCount / duration : 0.0);

                m_totalShootingTime += duration;
                m_averageShotsPerSecond = m_totalShootingTime > 0 ? m_totalShots / m_totalShootingTime : 0.0;
            }
        }

        // --- Always-published telemetry ---
        double currentElapsed = now - m_setStartTime;
        double currentShotsPerSecond = (m_feederActive && currentElapsed > 0) ? m_shotCount / currentElapsed : 0.0;

        Logger.recordOutput("ShotLogger/Feeder Active", m_feederActive);
        Logger.recordOutput("ShotLogger/Current Set Shots", m_feederActive ? m_shotCount : 0);
        Logger.recordOutput("ShotLogger/Current Shots Per Second", currentShotsPerSecond);
        Logger.recordOutput("ShotLogger/Total Sets", m_setNumber);
        Logger.recordOutput("ShotLogger/Total Shots", m_totalShots);
        Logger.recordOutput("ShotLogger/Total Shooting Time (s)", m_totalShootingTime);
        Logger.recordOutput("ShotLogger/Average Shots Per Second", m_averageShotsPerSecond);


    }
}
