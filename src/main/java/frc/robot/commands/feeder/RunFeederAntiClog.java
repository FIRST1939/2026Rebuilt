package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.spindexer.Spindexer;

public class RunFeederAntiClog extends RepeatCommand {
    
    public RunFeederAntiClog (Feeder feeder, Spindexer spindexer) {

        super(Commands.sequence(
            new RunFeederVelocity(feeder, FeederConstants.kFeederVelocity).until(() -> spindexer.isClogged()),
            new RunFeederVelocity(feeder, FeederConstants.kFeederReverseVelocity).withTimeout(0.375)
        ));
    }
}
