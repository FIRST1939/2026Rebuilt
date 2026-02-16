// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.shooter.*;

import frc.robot.commands.spindexer.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.shooter.*;

public class RobotContainer {

    private final Intake m_intake;
    private final Spindexer m_spindexer;
    private final Feeder m_feeder;
    private final Shooter m_shooter;

    private enum OpModes {
        MATCH,
        PERCENT,
        INTAKE_CHARACTERIZATION,
        SPINDEXER_CHARACTERIZATION,
        FEEDER_CHARACTERIZATION,
        SHOOTER_CHARACTERIZATION
    }

    private final LoggedDashboardChooser<OpModes> m_opModeSelector = new LoggedDashboardChooser<>("Op Mode Selector");
    private final LoggedNetworkNumber m_controllerSetpoint = new LoggedNetworkNumber("Controller Setpoint");
    private final LoggedNetworkNumber m_updateFeedbackP = new LoggedNetworkNumber("Feedback P");
    private final LoggedNetworkNumber m_updateFeedbackD = new LoggedNetworkNumber("Feedback D");
    private final LoggedNetworkNumber m_updateProfileCruiseVelocity = new LoggedNetworkNumber("Profile Cruise Velocity");
    private final LoggedNetworkNumber m_updateProfileMaxAcceleration = new LoggedNetworkNumber("Profile Max Acceleration");
    private final LoggedNetworkNumber m_updateProfileAllowedError = new LoggedNetworkNumber("Profile Allowed Error");

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer(boolean isReal) {

        if (isReal) {

            m_intake = new Intake(new IntakeIOHardware());
            m_spindexer = new Spindexer(new SpindexerIOHardware());
            m_feeder = new Feeder(new FeederIOHardware());
            m_shooter = new Shooter(new ShooterIOHardware());
        } else {

            m_intake = new Intake(new IntakeIOSim());
            m_spindexer = new Spindexer(new SpindexerIOSim());
            m_feeder = new Feeder(new FeederIOSim());
            m_shooter = new Shooter(new ShooterIOHardware());
        }
        
        m_opModeSelector.addDefaultOption("Match", OpModes.MATCH);
        m_opModeSelector.addOption("Percent", OpModes.PERCENT);
        m_opModeSelector.addOption("Intake Characterization", OpModes.INTAKE_CHARACTERIZATION);
        m_opModeSelector.addOption("Spindexer Characterization", OpModes.SPINDEXER_CHARACTERIZATION);
        m_opModeSelector.addOption("Feeder Characterization", OpModes.FEEDER_CHARACTERIZATION);
        m_opModeSelector.addOption("Shooter Characterization", OpModes.SHOOTER_CHARACTERIZATION);

        configureBindings();
    }

    private void configureBindings() {

        Trigger percentMode = new Trigger(() -> m_opModeSelector.get() == OpModes.PERCENT);
        Trigger intakeCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.INTAKE_CHARACTERIZATION);
        Trigger spindexerCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.SPINDEXER_CHARACTERIZATION);
        Trigger feederCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.FEEDER_CHARACTERIZATION);
        Trigger shooterCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.SHOOTER_CHARACTERIZATION);

        percentMode.and(m_driverController.a()).whileTrue(new RunSpindexer(m_spindexer, -1.0));
        percentMode.and(m_driverController.x()).whileTrue(new RunFeeder(m_feeder, 1.0));
        percentMode.and(m_driverController.y()).whileTrue(new RunShooter(m_shooter, 0.6));

        intakeCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_intake.rollerSysIdQuasistaticForward());
        intakeCharacterizationMode.and(m_driverController.rightBumper()).whileTrue(m_intake.rollerSysIdQuasistaticReverse());
        intakeCharacterizationMode.and(m_driverController.leftTrigger()).whileTrue(m_intake.rollerSysIdDynamicForward());
        intakeCharacterizationMode.and(m_driverController.rightTrigger()).whileTrue(m_intake.rollerSysIdDynamicReverse());

        intakeCharacterizationMode.and(m_driverController.povUp()).whileTrue(m_intake.leftPivotSysIdQuasistaticForward());
        intakeCharacterizationMode.and(m_driverController.povRight()).whileTrue(m_intake.leftPivotSysIdQuasistaticReverse());
        intakeCharacterizationMode.and(m_driverController.povDown()).whileTrue(m_intake.leftPivotSysIdDynamicForward());
        intakeCharacterizationMode.and(m_driverController.povLeft()).whileTrue(m_intake.leftPivotSysIdDynamicReverse());

        intakeCharacterizationMode.and(m_driverController.y()).whileTrue(m_intake.rightPivotSysIdQuasistaticForward());
        intakeCharacterizationMode.and(m_driverController.b()).whileTrue(m_intake.rightPivotSysIdQuasistaticReverse());
        intakeCharacterizationMode.and(m_driverController.a()).whileTrue(m_intake.rightPivotSysIdDynamicForward());
        intakeCharacterizationMode.and(m_driverController.x()).whileTrue(m_intake.rightPivotSysIdDynamicReverse());

        intakeCharacterizationMode.and(m_driverController.leftStick()).onTrue(Commands.sequence(
            Commands.runOnce(() -> {
                m_intake.updateRollerControllerFeedback(
                    m_updateFeedbackP.getAsDouble(),
                    m_updateFeedbackD.getAsDouble()
                );
            }),
            Commands.runOnce(() -> {
                m_intake.setRollerVelocity(m_controllerSetpoint.getAsDouble());
            }, m_intake)
        ));

        intakeCharacterizationMode.and(m_driverController.rightStick()).onTrue(Commands.sequence(
            Commands.runOnce(() -> {
                m_intake.updatePivotControllerFeedback(
                    m_updateFeedbackP.getAsDouble(),
                    m_updateFeedbackD.getAsDouble()
                );
            }),
            Commands.runOnce(() -> {
                m_intake.updatePivotControllerProfile(
                    m_updateProfileCruiseVelocity.getAsDouble(),
                    m_updateProfileMaxAcceleration.getAsDouble(),
                    m_updateProfileAllowedError.getAsDouble()
                );
            }),
            Commands.runOnce(() -> {
                m_intake.setPivotPosition(m_controllerSetpoint.getAsDouble());
            }, m_intake)
        ));

        spindexerCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_spindexer.sysIdQuasistaticForward());
        spindexerCharacterizationMode.and(m_driverController.rightBumper()).whileTrue(m_spindexer.sysIdQuasistaticReverse());
        spindexerCharacterizationMode.and(m_driverController.leftTrigger()).whileTrue(m_spindexer.sysIdDynamicForward());
        spindexerCharacterizationMode.and(m_driverController.rightTrigger()).whileTrue(m_spindexer.sysIdDynamicReverse());

        feederCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_feeder.sysIdQuasistaticForward());
        feederCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_feeder.sysIdQuasistaticReverse());
        feederCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_feeder.sysIdDynamicForward());
        feederCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_feeder.sysIdDynamicReverse());

        feederCharacterizationMode.and(m_driverController.leftStick()).onTrue(Commands.sequence(
            Commands.runOnce(() -> {
                m_feeder.updateControllerFeedback(
                    m_updateFeedbackP.getAsDouble(),
                    m_updateFeedbackD.getAsDouble()
                );
            }),
            Commands.runOnce(() -> {
                m_feeder.setFeederVelocity(m_controllerSetpoint.getAsDouble());
            }, m_feeder)
        ));

        shooterCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_shooter.flywheelSysIdQuasistaticForward());
        shooterCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_shooter.flywheelSysIdQuasistaticReverse());
        shooterCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_shooter.flywheelSysIdDynamicForward());
        shooterCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_shooter.flywheelSysIdDynamicReverse());

        shooterCharacterizationMode.and(m_driverController.y()).whileTrue(m_shooter.hoodSysIdQuasistaticForward());
        shooterCharacterizationMode.and(m_driverController.b()).whileTrue(m_shooter.hoodSysIdQuasistaticReverse());
        shooterCharacterizationMode.and(m_driverController.a()).whileTrue(m_shooter.hoodSysIdDynamicForward());
        shooterCharacterizationMode.and(m_driverController.x()).whileTrue(m_shooter.hoodSysIdDynamicReverse());

        shooterCharacterizationMode.and(m_driverController.leftStick()).onTrue(Commands.sequence(
            Commands.runOnce(() -> {
                m_shooter.updateFlywheelControllerFeedback(
                    m_updateFeedbackP.getAsDouble(),
                    m_updateFeedbackD.getAsDouble()
                );
            }),
            Commands.runOnce(() -> {
                m_shooter.updateFlywheelControllerProfile(
                    m_updateProfileMaxAcceleration.getAsDouble(),
                    m_updateProfileAllowedError.getAsDouble()
                );
            }),
            Commands.runOnce(() -> {
                m_shooter.setFlywheelVelocity(m_controllerSetpoint.getAsDouble());
            }, m_shooter)
        ));

        shooterCharacterizationMode.and(m_driverController.rightStick()).onTrue(Commands.sequence(
            Commands.runOnce(() -> {
                m_shooter.updateHoodControllerFeedback(
                    m_updateFeedbackP.getAsDouble(),
                    m_updateFeedbackD.getAsDouble()
                );
            }),
            Commands.runOnce(() -> {
                m_shooter.setHoodPosition(m_controllerSetpoint.getAsDouble());
            }, m_intake)
        ));
    }

    public Command getAutonomousCommand() {
      
        return Commands.print("No autonomous command configured");
    }

    public void simulateBatteryLoad() {

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                m_intake.getRollerCurrent(),
                m_spindexer.getSpindexerCurrent(),
                m_feeder.getFeederCurrent()
            )
        );
    }
}
