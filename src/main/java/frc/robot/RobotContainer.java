// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
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
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOHardware;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.shooter.*;

import frc.robot.commands.spindexer.*;
import frc.robot.commands.climber.SetClimberClimbingPosition;
import frc.robot.commands.climber.SetClimberRaisingPosition;
import frc.robot.commands.feeder.*;
import frc.robot.commands.intake.PivotIntake;
import frc.robot.commands.intake.PivotAndRunIntake;
import frc.robot.commands.intake.PivotIntakePercentage;
import frc.robot.commands.intake.RunIntakeRollerPercentage;
import frc.robot.commands.shooter.*;

public class RobotContainer {

    private final Intake m_intake;
    private final Spindexer m_spindexer;
    private final Feeder m_feeder;
    private final Shooter m_shooter;
    private final Climber m_climber;

    private enum OpModes {
        MATCH,
        PERCENT,
        INTAKE_CHARACTERIZATION,
        SPINDEXER_CHARACTERIZATION,
        FEEDER_CHARACTERIZATION,
        SHOOTER_CHARACTERIZATION,
        CLIMBER_CHARACTERIZATION
    }

    private final LoggedDashboardChooser<OpModes> m_opModeSelector = new LoggedDashboardChooser<>("Op Mode Selector");

    private final LoggedNetworkNumber m_controllerSetpoint = new LoggedNetworkNumber("/Tuning/Controller Setpoint", 0);
    private final LoggedNetworkNumber m_updateFeedbackP = new LoggedNetworkNumber("/Tuning/Feedback P", 0);
    private final LoggedNetworkNumber m_updateFeedbackD = new LoggedNetworkNumber("/Tuning/Feedback D", 0);
    private final LoggedNetworkNumber m_updateFeedbackP2 = new LoggedNetworkNumber("/Tuning/Feedback P2", 0);
    private final LoggedNetworkNumber m_updateFeedbackD2 = new LoggedNetworkNumber("/Tuning/Feedback D2", 0);
    private final LoggedNetworkNumber m_updateProfileCruiseVelocity = new LoggedNetworkNumber("/Tuning/Profile Cruise Velocity", 0);
    private final LoggedNetworkNumber m_updateProfileMaxAcceleration = new LoggedNetworkNumber("/Tuning/Profile Max Acceleration", 0);
    private final LoggedNetworkNumber m_updateProfileAllowedError = new LoggedNetworkNumber("/Tuning/Profile Allowed Error", 0);
    private DoubleSupplier m_controllerErrorSupplier = () -> 0.0;
    private DoubleSupplier m_controllerErrorSupplier2 = () -> 0.0;

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer(boolean isReal) {

        if (isReal) {

            m_intake = new Intake(new IntakeIOHardware());
            m_spindexer = new Spindexer(new SpindexerIOHardware());
            m_feeder = new Feeder(new FeederIOHardware());
            m_shooter = new Shooter(new ShooterIOHardware());
            m_climber = new Climber(new ClimberIOHardware());
        } else {

            m_intake = new Intake(new IntakeIOSim());
            m_spindexer = new Spindexer(new SpindexerIOSim());
            m_feeder = new Feeder(new FeederIOSim());
            m_shooter = new Shooter(new ShooterIOHardware());
            m_climber = new Climber(new ClimberIOHardware());
        }
        
        m_opModeSelector.addDefaultOption("Match", OpModes.MATCH);
        m_opModeSelector.addOption("Percent", OpModes.PERCENT);
        m_opModeSelector.addOption("Intake Characterization", OpModes.INTAKE_CHARACTERIZATION);
        m_opModeSelector.addOption("Spindexer Characterization", OpModes.SPINDEXER_CHARACTERIZATION);
        m_opModeSelector.addOption("Feeder Characterization", OpModes.FEEDER_CHARACTERIZATION);
        m_opModeSelector.addOption("Shooter Characterization", OpModes.SHOOTER_CHARACTERIZATION);
        m_opModeSelector.addOption("Climber Characterization", OpModes.CLIMBER_CHARACTERIZATION);

        configureMatchBindings();
        configurePercentBindings();
        configureIntakeCharacterizationBindings();
        configureSpindexerCharacterizationBindings();
        configureFeederCharacterizationBindings();
        configureShooterCharacterizationBindings();
        configureClimberCharacterizationBindings();
    }

    private void configureMatchBindings () {

        Trigger matchMode = new Trigger(() -> m_opModeSelector.get() == OpModes.MATCH);

        matchMode.and(m_driverController.y()).whileTrue(new RunFeederVelocity(m_feeder, Constants.kFeederVelocity)); //Run Feeder
        //matchMode.and(m_driverController.b()).whileTrue(new PivotAndRunIntake(m_intake, Constants.kPivotOutSetpoint, () -> 0.0)); //Pivot Out and Run Intake
        matchMode.and(m_driverController.b()).onTrue(new PivotIntake(m_intake, Constants.kPivotOutSetpoint));
        matchMode.and(m_driverController.a()).onTrue(new PivotIntake(m_intake, Constants.kPivotInSetpoint)); //Pivot Intake In
        matchMode.and(m_driverController.x()).whileTrue(new RunSpindexerVelocity(m_spindexer, Constants.kSpindexerVelocity)); //Run Spindexer

        matchMode.and(m_driverController.leftBumper()).whileTrue(new RunFlywheelAndHood(m_shooter, () -> 0.0, () -> 0.0));
        matchMode.and(m_driverController.rightBumper()).whileTrue(new SetClimberClimbingPosition(m_climber, Constants.kClimberClimbingSetpoint));
        matchMode.and(m_driverController.leftTrigger()).whileTrue(new SetClimberRaisingPosition(m_climber, Constants.kClimberRaisingSetpoint));
    }

    private void configurePercentBindings() {

        Trigger percentMode = new Trigger(() -> m_opModeSelector.get() == OpModes.PERCENT);

        percentMode.and(m_driverController.a()).whileTrue(new RunSpindexerPercentage(m_spindexer, 0.8));
        percentMode.and(m_driverController.x()).whileTrue(new RunFeederPercentage(m_feeder, 0.8));
        percentMode.and(m_driverController.y()).whileTrue(new RunFlywheelPercentage(m_shooter, 0.3));
        percentMode.and(m_driverController.b()).whileTrue(new RunHoodPercentage(m_shooter, -0.2));
        percentMode.and(m_driverController.leftTrigger()).whileTrue(new RunFlywheelPercentage(m_shooter, 0.55));
        percentMode.and(m_driverController.rightTrigger()).whileTrue(new RunIntakeRollerPercentage(m_intake, 0.225));
        percentMode.and(m_driverController.leftBumper()).whileTrue(new PivotIntakePercentage(m_intake, -0.25));
        percentMode.and(m_driverController.rightBumper()).whileTrue(new PivotIntakePercentage(m_intake, 0.25));
    }

    public void configureIntakeCharacterizationBindings () {

        Trigger intakeCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.INTAKE_CHARACTERIZATION);

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


        intakeCharacterizationMode.and(m_driverController.leftStick()).onFalse(Commands.runOnce(() -> {

            m_intake.setRollerPercentage(0);
        }, m_intake));

        intakeCharacterizationMode.and(m_driverController.rightStick()).onTrue(Commands.runOnce(() -> {

            m_intake.updateLeftPivotControllerFeedback(
                m_updateFeedbackP.getAsDouble(),
                m_updateFeedbackD.getAsDouble()
            );

            m_intake.updateRightPivotControllerFeedback(
                m_updateFeedbackP2.getAsDouble(),
                m_updateFeedbackD2.getAsDouble()
            );

            m_intake.updatePivotControllerProfile(
                m_updateProfileCruiseVelocity.getAsDouble(),
                m_updateProfileMaxAcceleration.getAsDouble(),
                m_updateProfileAllowedError.getAsDouble()
            );

            double setpoint = m_controllerSetpoint.getAsDouble();
            m_controllerErrorSupplier = () -> setpoint - m_intake.getLeftPivotPosition();
            m_controllerErrorSupplier2 = () -> setpoint - m_intake.getRightPivotPosition();
            m_intake.setPivotPosition(setpoint);
        }, m_intake));

        intakeCharacterizationMode.and(m_driverController.rightStick()).onFalse(Commands.runOnce(() -> {

            m_intake.setPivotPercentage(0);
        }, m_intake));
    }

    public void configureSpindexerCharacterizationBindings () {

        Trigger spindexerCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.SPINDEXER_CHARACTERIZATION);

        spindexerCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_spindexer.sysIdQuasistaticForward());
        spindexerCharacterizationMode.and(m_driverController.rightBumper()).whileTrue(m_spindexer.sysIdQuasistaticReverse());
        spindexerCharacterizationMode.and(m_driverController.leftTrigger()).whileTrue(m_spindexer.sysIdDynamicForward());
        spindexerCharacterizationMode.and(m_driverController.rightTrigger()).whileTrue(m_spindexer.sysIdDynamicReverse());
    }

    public void configureFeederCharacterizationBindings () {

        Trigger feederCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.FEEDER_CHARACTERIZATION);

        feederCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_feeder.sysIdQuasistaticForward());
        feederCharacterizationMode.and(m_driverController.rightBumper()).whileTrue(m_feeder.sysIdQuasistaticReverse());
        feederCharacterizationMode.and(m_driverController.leftTrigger()).whileTrue(m_feeder.sysIdDynamicForward());
        feederCharacterizationMode.and(m_driverController.rightTrigger()).whileTrue(m_feeder.sysIdDynamicReverse());

        feederCharacterizationMode.and(m_driverController.povUp()).onTrue(Commands.runOnce(() -> {

            m_feeder.updateControllerFeedback(
                m_updateFeedbackP.getAsDouble(),
                m_updateFeedbackD.getAsDouble()
            );

            double setpoint = m_controllerSetpoint.getAsDouble();
            m_controllerErrorSupplier = () -> setpoint - m_feeder.getFeederVelocity();
            m_feeder.setFeederVelocity(setpoint);
        }, m_feeder));

        feederCharacterizationMode.and(m_driverController.povUp()).onFalse(Commands.runOnce(() -> {

            m_feeder.setFeederPercentage(0);
        }, m_feeder));
    }

    public void configureShooterCharacterizationBindings () {

        Trigger shooterCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.SHOOTER_CHARACTERIZATION);

        shooterCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_shooter.flywheelSysIdQuasistaticForward());
        shooterCharacterizationMode.and(m_driverController.rightBumper()).whileTrue(m_shooter.flywheelSysIdQuasistaticReverse());
        shooterCharacterizationMode.and(m_driverController.leftTrigger()).whileTrue(m_shooter.flywheelSysIdDynamicForward());
        shooterCharacterizationMode.and(m_driverController.rightTrigger()).whileTrue(m_shooter.flywheelSysIdDynamicReverse());

        shooterCharacterizationMode.and(m_driverController.y()).whileTrue(m_shooter.hoodSysIdQuasistaticForward());
        shooterCharacterizationMode.and(m_driverController.b()).whileTrue(m_shooter.hoodSysIdQuasistaticReverse());
        shooterCharacterizationMode.and(m_driverController.a()).whileTrue(m_shooter.hoodSysIdDynamicForward());
        shooterCharacterizationMode.and(m_driverController.x()).whileTrue(m_shooter.hoodSysIdDynamicReverse());

        shooterCharacterizationMode.and(m_driverController.povRight()).whileTrue(Commands.parallel(
            new RunSpindexerPercentage(m_spindexer, 0.8),
            new RunFeederPercentage(m_feeder, 0.8)
        ));

        shooterCharacterizationMode.and(m_driverController.povLeft()).onTrue(Commands.runOnce(() -> {

            m_shooter.updateFlywheelControllerFeedback(
                m_updateFeedbackP.getAsDouble(),
                m_updateFeedbackD.getAsDouble()
            );

            double setpoint = m_controllerSetpoint.getAsDouble();
            m_controllerErrorSupplier = () -> setpoint - m_shooter.getFlywheelVelocity();
            m_shooter.setFlywheelVelocity(setpoint);
        }, m_shooter));

        shooterCharacterizationMode.and(m_driverController.povLeft()).onFalse(Commands.runOnce(() -> {

            m_shooter.setFlywheelPercentage(0);
        }, m_shooter));

        shooterCharacterizationMode.and(m_driverController.povRight()).onTrue(Commands.runOnce(() -> {

            m_shooter.updateHoodControllerFeedback(
                m_updateFeedbackP.getAsDouble(),
                m_updateFeedbackD.getAsDouble()
            );

            double setpoint = m_controllerSetpoint.getAsDouble();
            m_controllerErrorSupplier = () -> setpoint - m_shooter.getHoodPosition();
            m_shooter.setHoodPosition(setpoint);
        }, m_intake));

        shooterCharacterizationMode.and(m_driverController.povRight()).onFalse(Commands.runOnce(() -> {

            m_shooter.setHoodPercentage(0);
        }, m_shooter));
    }

    private void configureClimberCharacterizationBindings () {

        Trigger climberCharacterizationMode = new Trigger(() -> m_opModeSelector.get() == OpModes.CLIMBER_CHARACTERIZATION);

        climberCharacterizationMode.and(m_driverController.leftBumper()).whileTrue(m_climber.raisingSysIdQuasistaticForward());
        climberCharacterizationMode.and(m_driverController.rightBumper()).whileTrue(m_climber.raisingSysIdQuasistaticReverse());
        climberCharacterizationMode.and(m_driverController.leftTrigger()).whileTrue(m_climber.raisingSysIdDynamicForward());
        climberCharacterizationMode.and(m_driverController.rightTrigger()).whileTrue(m_climber.raisingSysIdDynamicReverse());

        climberCharacterizationMode.and(m_driverController.y()).whileTrue(m_climber.climbingSysIdQuasistaticForward());
        climberCharacterizationMode.and(m_driverController.b()).whileTrue(m_climber.climbingSysIdQuasistaticReverse());
        climberCharacterizationMode.and(m_driverController.a()).whileTrue(m_climber.climbingSysIdDynamicForward());
        climberCharacterizationMode.and(m_driverController.x()).whileTrue(m_climber.climbingSysIdDynamicReverse());

        climberCharacterizationMode.and(m_driverController.povUp()).onTrue(Commands.runOnce(() -> {

            m_climber.updateRaisingControllerFeedback(
                m_updateFeedbackP.getAsDouble(),
                m_updateFeedbackD.getAsDouble()
            );

            m_climber.updateControllerProfile(
                m_updateProfileCruiseVelocity.getAsDouble(),
                m_updateProfileMaxAcceleration.getAsDouble(),
                m_updateProfileAllowedError.getAsDouble()
            );

            double setpoint = m_controllerSetpoint.getAsDouble();
            m_controllerErrorSupplier = () -> m_climber.getControllerSetpoint() - m_climber.getClimberPosition();
            m_climber.setRaisingPosition(setpoint);
        }, m_climber));

        climberCharacterizationMode.and(m_driverController.povUp()).onFalse(Commands.runOnce(() -> {

            m_climber.setClimberPercentage(0);
        }, m_climber));

        climberCharacterizationMode.and(m_driverController.povDown()).onTrue(Commands.runOnce(() -> {

            m_climber.updateClimbingControllerFeedback(
                m_updateFeedbackP.getAsDouble(),
                m_updateFeedbackD.getAsDouble()
            );

            m_climber.updateControllerProfile(
                m_updateProfileCruiseVelocity.getAsDouble(),
                m_updateProfileMaxAcceleration.getAsDouble(),
                m_updateProfileAllowedError.getAsDouble()
            );

            double setpoint = m_controllerSetpoint.getAsDouble();
            m_controllerErrorSupplier = () -> m_climber.getControllerSetpoint() - m_climber.getClimberPosition();
            m_climber.setClimbingPosition(setpoint);
        }, m_climber));

        climberCharacterizationMode.and(m_driverController.povDown()).onFalse(Commands.runOnce(() -> {

            m_climber.setClimberPercentage(0);
        }, m_climber));
    }

    public void logControllerError () {

        Logger.recordOutput("Controller Error", m_controllerErrorSupplier.getAsDouble());
        Logger.recordOutput("Controller Error 2", m_controllerErrorSupplier2.getAsDouble());
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
