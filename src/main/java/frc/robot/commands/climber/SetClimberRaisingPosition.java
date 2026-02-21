package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class SetClimberRaisingPosition extends Command {

    private final Climber m_climber;
    private final double m_position;

    public SetClimberRaisingPosition(Climber climber, double position) {
        m_climber = climber;
        m_position = position;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.setRaisingPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}