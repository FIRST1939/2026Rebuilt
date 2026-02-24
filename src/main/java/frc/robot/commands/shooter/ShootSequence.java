package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.feeder.StartFeederVelocity;
import frc.robot.commands.feeder.RunFeederVelocity;
import frc.robot.commands.spindexer.RunSpindexerVelocity;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;


public final class ShootSequence {

    private ShootSequence() {} // prevent instantiation
    

    public static Command create(
            Shooter shooter,
            Feeder feeder,
            Spindexer spindexer,
            double feederRPM,
            double feederThreshold,
            double spindexerRPM) {

        // The shooter is NOT required here — it keeps running from its own command.
        return Commands.sequence(
            Commands.waitUntil(() -> shooter.isAtGoal()),
            new StartFeederVelocity(feeder, feederRPM, feederThreshold),
            Commands.parallel(
                new RunFeederVelocity(feeder, feederRPM),
                new RunSpindexerVelocity(spindexer, spindexerRPM)
            )
        ).finallyDo(() -> {
            feeder.setFeederPercentage(0.0);
            spindexer.setSpindexerPercentage(0.0);
        });
    }
}
