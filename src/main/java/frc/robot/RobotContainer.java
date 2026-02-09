// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Spindexer.*;
import frc.robot.commands.feeder.*;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOHardware;

public class RobotContainer {
  private final Spindexer spindexer = new Spindexer (new SpindexerIOHardware());
  private final Feeder feeder = new Feeder (new FeederIOHardware());
  private final Intake intake = new Intake(new IntakeIOHardware());
  private final RunSpindexer spindexerRunOutput = new RunSpindexer(spindexer, 2400);
  private final RunFeeder feederRunOutput = new RunFeeder (feeder, 2400);
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    m_driverController.square().whileTrue(spindexer.SpindexerSysIdQuasistaticForward());
    m_driverController.circle().whileTrue(spindexer.SpindexerSysIdQuasistaticReverse());
    m_driverController.triangle().whileTrue(spindexer.SpindexerSysIdDynamicForward());
    m_driverController.cross().whileTrue(spindexer.SpindexerSysIdDynamicReverse());

    m_driverController.R2().whileTrue(intake.RollerSysIdQuasistaticForward());
    m_driverController.L2().whileTrue(intake.RollerSysIdQuasistaticReverse());
    m_driverController.R1().whileTrue(intake.RollerSysIdDynamicForward());
    m_driverController.L1().whileTrue(intake.RollerSysIdDynamicReverse());
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
