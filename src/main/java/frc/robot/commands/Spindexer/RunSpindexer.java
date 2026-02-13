// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Spindexer;

import frc.robot.subsystems.spindexer.*;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunSpindexer extends Command {
  private final Spindexer m_subsystem;
  private final double m_percentage;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunSpindexer(Spindexer subsystem, double percentage) {
    m_subsystem = subsystem;
    m_percentage = percentage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
     m_subsystem.setSpindexerPercentage(m_percentage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_subsystem.setSpindexerPercentage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
