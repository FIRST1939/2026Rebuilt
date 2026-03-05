// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Util;

public class Robot extends LoggedRobot {

    private final RobotContainer m_robotContainer;
    private Command m_autonomousCommand;

    public Robot() {

        Logger.recordMetadata("ProjectName", "2026Rebuilt");

        if (isReal()) {

            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {

            // TODO Replay
            setUseTiming(false);
            Logger.addDataReceiver(new NT4Publisher());
            //Logger.setReplaySource(new WPILOGReader(logPath));
            //Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        Logger.start();
        
        m_robotContainer = new RobotContainer(isReal());
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();

        m_robotContainer.updateShotSolution();
        m_robotContainer.logControllerError();

        if (isSimulation()) {

            m_robotContainer.simulateBatteryLoad();
        }

        updateActiveDisplay();
    }

    public void updateActiveDisplay () {

        double matchTime = DriverStation.getMatchTime();

        /*
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) { return; }

        boolean redActiveFirst;

        switch (gameData.charAt(0)) { // Who Won Auto
            case 'R' -> redActiveFirst = false;
            case 'B' -> redActiveFirst = true;
            default -> { return; }
        }

        boolean firstActiveShift = Util.isRedAlliance() ? redActiveFirst : !redActiveFirst;
        */

        if (matchTime > 140) {

            Logger.recordOutput("Shift Timer", matchTime - 140);
        } else if (matchTime > 130) { // Transition Shift

            Logger.recordOutput("Shift Timer", matchTime - 130);
        } else if (matchTime > 105) { // Shift 1

            Logger.recordOutput("Shift Timer", matchTime - 105);
        } else if (matchTime > 80) { // Shift 2

            Logger.recordOutput("Shift Timer", matchTime - 80);
        } else if (matchTime > 55) { // Shift 3

            Logger.recordOutput("Shift Timer", matchTime - 105);
        } else if (matchTime > 30) { // Shift 4

            Logger.recordOutput("Shift Timer", matchTime - 30);
        } else { // Endgame

            Logger.recordOutput("Shift Timer", matchTime);
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {

            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {

            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
