// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Spindexer.RunSpindexer;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.feeder.*;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.spindexer.*;

public class RobotContainer {

   // private final Intake m_intake;
    private final Spindexer m_spindexer;
    private final Feeder m_feeder;
    private final Shooter m_shooter;
    private final RunSpindexer c_spindexer;
    private final RunFeeder c_feeder;
    private final RunShooter c_shooter;

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer(boolean isReal) {

        if (isReal) {

          //  m_intake = new Intake(new IntakeIOHardware());
            m_spindexer = new Spindexer(new SpindexerIOHardware());
            m_feeder = new Feeder(new FeederIOHardware());
            m_shooter = new Shooter(new ShooterIOHardware());
        } else {
            m_shooter = new Shooter(new ShooterIOHardware());
          //  m_intake = new Intake(new IntakeIOSim());
            m_spindexer = new Spindexer(new SpindexerIOSim());
            m_feeder = new Feeder(new FeederIOSim());
        }

        c_spindexer = new RunSpindexer(m_spindexer, -1);
        c_feeder = new RunFeeder(m_feeder, 1);
        c_shooter = new RunShooter(m_shooter, 0.6);
        
        configureBindings();
    }

    private void configureBindings() {

      System.out.println("config working");

        m_driverController.a().whileTrue(c_spindexer);
        m_driverController.x().whileTrue(c_feeder);
        m_driverController.y().whileTrue(c_shooter);
       // m_driverController.y().whileTrue(m_spindexer.SpindexerSysIdDynamicForward());
       // m_driverController.b).whileTrue(m_spindexer.SpindexerSysIdDynamicReverse());

        //m_driverController.leftBumper().whileTrue(m_intake.RollerSysIdQuasistaticForward());
        //m_driverController.rightBumper().whileTrue(m_intake.RollerSysIdQuasistaticReverse());
        //m_driverController.leftTrigger().whileTrue(m_intake.RollerSysIdDynamicForward());
        //m_driverController.rightTrigger().whileTrue(m_intake.RollerSysIdDynamicReverse());
    }

    public Command getAutonomousCommand() {
      
        return Commands.print("No autonomous command configured");
    }

    public void simulateBatteryLoad() {

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
               // m_intake.getRollerCurrent(),
                m_spindexer.getSpindexerCurrent()
                // m_feeder.getFeederCurrent()
            )
        );
    }
}
