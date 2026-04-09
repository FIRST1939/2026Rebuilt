package frc.robot.commands.spindexer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.spindexer.Spindexer;

public class RunSpindexerAntiClog extends RepeatCommand {
    
    public RunSpindexerAntiClog (Spindexer spindexer) {

        super(Commands.sequence(
            new RunSpindexerPercentage(spindexer, SpindexerConstants.kSpindexerPercentage).until(() -> spindexer.isClogged()),
            new RunSpindexerPercentage(spindexer, SpindexerConstants.kSpindexerReversePercentage).withTimeout(0.5)
        ));
    }
}
