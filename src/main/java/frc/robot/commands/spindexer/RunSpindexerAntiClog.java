package frc.robot.commands.spindexer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.subsystems.spindexer.Spindexer;

public class RunSpindexerAntiClog extends RepeatCommand {
    
    public RunSpindexerAntiClog (Spindexer spindexer) {

        super(Commands.sequence(
            new RunSpindexerVelocity(spindexer, Constants.kSpindexerVelocity).until(() -> spindexer.isClogged()),
            new RunSpindexerVelocity(spindexer, Constants.kSpindexerReverseVelocity).withTimeout(0.5)
        ));
    }
}
