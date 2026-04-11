// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.MatchClock;
import frc.robot.util.ShiftUtil;
import frc.robot.util.ShiftUtil.Shift;

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
            //setUseTiming(false);
            Logger.addDataReceiver(new NT4Publisher());
            //Logger.setReplaySource(new WPILOGReader(logPath));
            //Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

            Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
            arena.setEfficiencyMode(false);
            SimulatedArena.overrideInstance(arena);
        }

        Logger.start();
        
        m_robotContainer = new RobotContainer(isReal());
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        MatchClock.syncFMS();

        m_robotContainer.updateShotSolutions();
        updateActiveDisplay();

        m_robotContainer.displayShotConditions();
        m_robotContainer.displayTargetHeading();
        m_robotContainer.displayRobotComponentsInAdvantageScope();
    }

    public void updateActiveDisplay () {

        Shift currentShift = ShiftUtil.getCurrentShift();
        Logger.recordOutput("Hub Active", currentShift.isActive());
        Logger.recordOutput("Current Shift", currentShift.getName());
        Logger.recordOutput("Remaining Shift Time", Math.round(currentShift.getSecondsRemaining() * 10.0) / 10.0);

        Optional<Boolean> activeFirst = ShiftUtil.activeFirst();
        Color autoDashboardColor;

        if (activeFirst.isEmpty()) { autoDashboardColor = new Color(253, 245, 60); } // Yelow
        else if (activeFirst.get()) { autoDashboardColor = new Color(244, 67, 54); } // Red
        else { autoDashboardColor = new Color(76, 175, 80); } // Green

        Logger.recordOutput("Did We Win Auto?", autoDashboardColor.toHexString());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {

        if (isSimulation()) {

            SimulatedArena.getInstance().resetFieldForAuto();
            m_robotContainer.simulateAutoPreload();
        }

        MatchClock.startAutonomousClock();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {

            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {

        MatchClock.stopClock();
        ShiftUtil.m_autoRun = true;
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {

            m_autonomousCommand.cancel();
        }

        MatchClock.startTeleopClock();
        m_robotContainer.clearIntakeState();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {

        MatchClock.stopClock();
    }

    @Override
    public void simulationPeriodic() {

        SimulatedArena.getInstance().simulationPeriodic();
        m_robotContainer.simulateBump();
        m_robotContainer.simulateIntakeBody();
        m_robotContainer.simulateShooting();
        
        m_robotContainer.displayFuelSimToAdvantageScope();
    }

    @Override
    public void testInit() {

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
