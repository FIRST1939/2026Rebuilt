// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bindings.*;
import frc.robot.bindings.characterization.*;
import frc.robot.bindings.config.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOHardware;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.*;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.intake.IntakeStateManager;

public class RobotContainer {

    private final Drive m_drive;
    private SwerveDriveSimulation m_swerveDriveSimulation = null;
    
    private final Intake m_intake;
    private final Spindexer m_spindexer;
    private final Feeder m_feeder;
    private final Shooter m_shooter;
    private final Climber m_climber;
    
    private final IntakeStateManager m_intakeStateManager;
    private final ShotSolver m_shotSolver;
    
    private enum OpModes {
        MATCH,
        PERCENT,
        INTAKE_CHARACTERIZATION,
        SPINDEXER_CHARACTERIZATION,
        FEEDER_CHARACTERIZATION,
        SHOOTER_CHARACTERIZATION,
        SHOT_CONFIG
    }
    
    private final LoggedDashboardChooser<OpModes> m_opModeSelector = new LoggedDashboardChooser<>("Op Mode Selector");
    private final LoggedDashboardChooser<Command> m_autoSelector;
    
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    
    public RobotContainer(boolean isReal) {
    
        if (isReal) {
    
            m_drive = new Drive(
                new GyroIOPigeon2(), 
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight), 
                new ModuleIOTalonFX(TunerConstants.BackLeft), 
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (Pose2d pose) -> {}
            );
    
            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, m_drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, m_drive::getRotation)
            );
    
            m_intake = new Intake(new IntakeIOHardware());
            m_spindexer = new Spindexer(new SpindexerIOHardware());
            m_feeder = new Feeder(new FeederIOHardware());
            m_shooter = new Shooter(new ShooterIOHardware());
            m_climber = new Climber(new ClimberIOHardware());
        } else {
    
            @SuppressWarnings("unchecked")
            DriveTrainSimulationConfig driveTrainSimulationConfig = new DriveTrainSimulationConfig(
                Pounds.of(121.1),
                Inches.of(34.25),
                Inches.of(34.25),
                Inches.of(21.75),
                Inches.of(21.75),
                COTS.ofPigeon2(),
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getKrakenX60(1),
                    TunerConstants.kDriveGearRatio,
                    TunerConstants.kSteerGearRatio,
                    TunerConstants.kDriveFrictionVoltage,
                    TunerConstants.kSteerFrictionVoltage,
                    TunerConstants.kWheelRadius,
                    TunerConstants.kSteerInertia,
                    1.2
                )
            );
    
            m_swerveDriveSimulation = new SwerveDriveSimulation(
                driveTrainSimulationConfig, 
                new Pose2d()
            );

            SimulatedArena.getInstance().addDriveTrainSimulation(m_swerveDriveSimulation);

            m_drive = new Drive(
                new GyroIOSim(m_swerveDriveSimulation.getGyroSimulation()), 
                new ModuleIOSim(m_swerveDriveSimulation.getModules()[0]), 
                new ModuleIOSim(m_swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(m_swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(m_swerveDriveSimulation.getModules()[3]),
                (Pose2d pose) -> m_swerveDriveSimulation.setSimulationWorldPose(pose)
            );

            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, m_swerveDriveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, m_swerveDriveSimulation::getSimulatedDriveTrainPose)
            );

            m_intake = new Intake(new IntakeIOSim());
            m_spindexer = new Spindexer(new SpindexerIOSim());
            m_feeder = new Feeder(new FeederIOSim());
            m_shooter = new Shooter(new ShooterIOSim());
            m_climber = new Climber(new ClimberIOSim());

            SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_intake.getRollerCurrent()));
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_intake.getLeftPivotCurrent()));
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_intake.getRightPivotCurrent()));
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_spindexer.getSpindexerCurrent()));
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_feeder.getFeederCurrent()));
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_shooter.getFlywheelLeaderCurrent()));
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_shooter.getFlywheelFollowerCurrent()));
            SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_shooter.getHoodCurrent()));
        }

        m_intakeStateManager = new IntakeStateManager(m_intake);
        m_shotSolver = new ShotSolver();

        m_opModeSelector.addDefaultOption("Match", OpModes.MATCH);
        m_opModeSelector.addOption("Percent", OpModes.PERCENT);

        m_opModeSelector.addOption("Intake Characterization", OpModes.INTAKE_CHARACTERIZATION);
        m_opModeSelector.addOption("Spindexer Characterization", OpModes.SPINDEXER_CHARACTERIZATION);
        m_opModeSelector.addOption("Feeder Characterization", OpModes.FEEDER_CHARACTERIZATION);
        m_opModeSelector.addOption("Shooter Characterization", OpModes.SHOOTER_CHARACTERIZATION);

        m_opModeSelector.addOption("Shot Config", OpModes.SHOT_CONFIG);

        BindingParams bindingParams = new BindingParams(
            m_drive, 
            m_intake, 
            m_spindexer, 
            m_feeder, 
            m_shooter, 
            m_climber, 
            m_intakeStateManager, 
            m_shotSolver, 
            m_driverController, 
            m_operatorController
        );

        new MatchBindings(bindingParams, new Trigger(() -> m_opModeSelector.get() == OpModes.MATCH));
        new PercentBindings(bindingParams, new Trigger(() -> m_opModeSelector.get() == OpModes.PERCENT));
        new PathPlannerBindings(bindingParams);

        new IntakeCharacterizationBindings(bindingParams, new Trigger(() -> m_opModeSelector.get() == OpModes.INTAKE_CHARACTERIZATION));
        new SpindexerCharacterizationBindings(bindingParams, new Trigger(() -> m_opModeSelector.get() == OpModes.SPINDEXER_CHARACTERIZATION));
        new FeederCharacterizationBindings(bindingParams, new Trigger(() -> m_opModeSelector.get() == OpModes.FEEDER_CHARACTERIZATION));
        new ShooterCharacterizationBindings(bindingParams, new Trigger(() -> m_opModeSelector.get() == OpModes.SHOOTER_CHARACTERIZATION));

        new ShotConfigBindings(bindingParams, new Trigger(() -> m_opModeSelector.get() == OpModes.SHOT_CONFIG));

        m_autoSelector = new LoggedDashboardChooser<>("Auto Selector", AutoBuilder.buildAutoChooser());
    }

    public Command getAutonomousCommand() {
      
        return m_autoSelector.get();
    }

    public void updateShotSolution() {

        m_shotSolver.calculateShotSolution(m_drive.getPose(), m_drive.getChassisSpeeds());
    }
    
    public void checkHubAlignment() {

        double error = Math.abs(m_drive.getRotation().getDegrees() - m_shotSolver.getShotSolution().aimHeading.getDegrees());
        Logger.recordOutput("Hub Aligned", error < 1.5);
    }

    public void displayFieldSimToAdvantageScope() {

        Logger.recordOutput("FieldSimulation/RobotPosition", m_swerveDriveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    public void displayRobotComponentsInAdvantageScope() {

        double backLinkZeroAngle = 27.570246;
        double backLinkLength = 0.177500;

        double frontLinkZeroAngle = 23.061811;
        double frontLinkLength = 0.169969;

        double intakeZeroAngle = 7.043217;
        double intakeHoleDistance = 0.127000;

        double backLinkAngle = Math.toRadians(backLinkZeroAngle + 103.5) - Rotations.of(m_intake.getPivotPosition()).in(Radians);

        double backLinkIntakeX = backLinkLength * Math.cos(backLinkAngle);
        double backLinkIntakeY = backLinkLength * Math.sin(backLinkAngle);

        double dx = backLinkIntakeX - intakeHoleDistance;
        double dy = backLinkIntakeY;

        double d = Math.hypot(dx, dy);

        double a = (Math.pow(frontLinkLength, 2) - Math.pow(intakeHoleDistance, 2) + Math.pow(d, 2)) / (2 * d);
        double h = Math.sqrt(Math.pow(frontLinkLength, 2) - Math.pow(a, 2));

        double Px = intakeHoleDistance + a * dx/d;
        double Py = a * dy/d;

        double frontLinkIntakeX = Px + h * dy/d;
        double frontLinkIntakeY = Py - h * dx/d;

        double frontLinkAngle = Math.atan2(frontLinkIntakeY, frontLinkIntakeX - intakeHoleDistance);
        double intakeAngle = Math.atan2(frontLinkIntakeY - backLinkIntakeY, frontLinkIntakeX - backLinkIntakeX);

        Pose3d intakePivotBack4Bar = new Pose3d(
            new Translation3d(
                0.050800,
                0.0,
                0.132733
            ),
            new Rotation3d(
                0.0,
                -backLinkAngle + Math.toRadians(backLinkZeroAngle),
                0.0
            )
        );

        Pose3d intakePivotFront4Bar = new Pose3d(
            new Translation3d(
                0.177800,
                0.0,
                0.132733
            ),
            new Rotation3d(
                0.0,
                -frontLinkAngle + Math.toRadians(frontLinkZeroAngle),
                0.0
            )
        );

        Pose3d intakePivot = new Pose3d(
            new Translation3d(
                (backLinkIntakeX + frontLinkIntakeX) / 2.0 + 0.050800,
                0.0,
                (backLinkIntakeY + frontLinkIntakeY) / 2.0 + 0.132733
            ),
            new Rotation3d(
                0.0,
                -intakeAngle - Math.toRadians(intakeZeroAngle),
                0.0
            )
        );

        Pose3d shooterHood = new Pose3d(
            new Translation3d(
                -0.209423,
                0.142875,
                0.413191
            ),
            new Rotation3d(
                0.0,
                Rotations.of(-m_shooter.getHoodPosition()).in(Radians),
                0.0
            )
        );

        Logger.recordOutput("FieldSimulation/RobotComponents", new Pose3d[] {intakePivotBack4Bar, intakePivotFront4Bar, intakePivot, shooterHood});
    }
}
