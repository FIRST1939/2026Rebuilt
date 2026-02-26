package frc.robot.bindings;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.commands.intake.PivotIntake;
import frc.robot.commands.intake.RunRoller;

public class QuickIntakeConfigBindings {

    
    private static final LoggedNetworkNumber m_deployedSetpoint =
            new LoggedNetworkNumber("/IntakeConfig/Deployed Setpoint", 0.225);
    private static final LoggedNetworkNumber m_idleSetpoint =
            new LoggedNetworkNumber("/IntakeConfig/Idle Setpoint", 0.12);
    private static final LoggedNetworkNumber m_storedSetpoint =
            new LoggedNetworkNumber("/IntakeConfig/Stored Setpoint", 0.02);

    
    private static final LoggedNetworkNumber m_rollerSpeed =
            new LoggedNetworkNumber("/IntakeConfig/Roller Speed", 1000);

    private static final LoggedNetworkNumber m_nudgeAmount =
            new LoggedNetworkNumber("/IntakeConfig/Nudge Amount", 0.005);

    private static final LoggedNetworkString m_activeSetpointDisplay =
            new LoggedNetworkString("/IntakeConfig/Active Setpoint", "DEPLOYED");


    
    private static final double PIVOT_MIN = 0.0;
    private static final double PIVOT_MAX = 0.3;

    
    private static enum ActiveSetpoint { DEPLOYED, IDLE, STORED }
    private static ActiveSetpoint m_activeSetpoint = ActiveSetpoint.DEPLOYED;

    public static void configure(
            Trigger intakeConfigMode,
            CommandXboxController controller,
            IntakePivot intakePivot,
            IntakeRollers intakeRollers) {

        // Right trigger: pivot intake to deployed setpoint (stays at position), run roller while held
        intakeConfigMode.and(controller.rightTrigger()).toggleOnTrue(
            Commands.runOnce(() -> { m_activeSetpoint = ActiveSetpoint.DEPLOYED; logSetpoints(); })
                .andThen(new PivotIntake(intakePivot, m_deployedSetpoint.getAsDouble()) {
                    @Override
                    public void initialize() {
                        intakePivot.setPivotPosition(m_deployedSetpoint.getAsDouble());
                    }
                })
        );

        intakeConfigMode.and(controller.rightTrigger()).whileTrue(
            new RunRoller(intakeRollers, m_rollerSpeed::getAsDouble)
        );

        // Right bumper: pivot intake to idle setpoint (stays at position), run roller inward while held
        intakeConfigMode.and(controller.rightBumper()).toggleOnTrue(
            Commands.runOnce(() -> { m_activeSetpoint = ActiveSetpoint.IDLE; logSetpoints(); })
                .andThen(new PivotIntake(intakePivot, m_idleSetpoint.getAsDouble()) {
                    @Override
                    public void initialize() {
                        intakePivot.setPivotPosition(m_idleSetpoint.getAsDouble());
                    }
                })
        );

        intakeConfigMode.and(controller.rightBumper()).whileTrue(
            new RunRoller(intakeRollers, () -> -m_rollerSpeed.getAsDouble())
        );

        // Back button: pivot intake to stored setpoint
        intakeConfigMode.and(controller.back()).onTrue(
            Commands.runOnce(() -> { m_activeSetpoint = ActiveSetpoint.STORED; logSetpoints(); })
                .andThen(new PivotIntake(intakePivot, m_storedSetpoint.getAsDouble()) {
                    @Override
                    public void initialize() {
                        intakePivot.setPivotPosition(m_storedSetpoint.getAsDouble());
                    }
                })
        );

        // Left bumper: nudge active setpoint up
        intakeConfigMode.and(controller.leftBumper()).onTrue(
            Commands.runOnce(() -> {
                nudgeActiveSetpoint(m_nudgeAmount.getAsDouble());
                intakePivot.setPivotPosition(getActiveSetpointValue());
            })
        );

        // Left trigger: nudge active setpoint down
        intakeConfigMode.and(controller.leftTrigger()).onTrue(
            Commands.runOnce(() -> {
                nudgeActiveSetpoint(-m_nudgeAmount.getAsDouble());
                intakePivot.setPivotPosition(getActiveSetpointValue());
            })
        );

        // POV up: nudge active setpoint up
        intakeConfigMode.and(controller.povUp()).onTrue(
            Commands.runOnce(() -> {
                nudgeActiveSetpoint(m_nudgeAmount.getAsDouble());
                intakePivot.setPivotPosition(getActiveSetpointValue());
            })
        );

        // POV down: nudge active setpoint down
        intakeConfigMode.and(controller.povDown()).onTrue(
            Commands.runOnce(() -> {
                nudgeActiveSetpoint(-m_nudgeAmount.getAsDouble());
                intakePivot.setPivotPosition(getActiveSetpointValue());
            })
        );

        // POV right: increase roller speed
        intakeConfigMode.and(controller.povRight()).onTrue(
            Commands.runOnce(() -> {
                m_rollerSpeed.set(m_rollerSpeed.getAsDouble() + 100);
                logSetpoints();
            })
        );

        // POV left: decrease roller speed
        intakeConfigMode.and(controller.povLeft()).onTrue(
            Commands.runOnce(() -> {
                m_rollerSpeed.set(Math.max(0, m_rollerSpeed.getAsDouble() - 100));
                logSetpoints();
            })
        );

        // Y button: run roller while held
        intakeConfigMode.and(controller.y()).whileTrue(
            new RunRoller(intakeRollers, m_rollerSpeed::getAsDouble)
        );
    }

    private static ActiveSetpoint getActiveSetpoint() {
        return m_activeSetpoint;
    }

    
    private static void nudgeActiveSetpoint(double delta) {
        switch (getActiveSetpoint()) {
            case DEPLOYED:
                m_deployedSetpoint.set(
                    MathUtil.clamp(m_deployedSetpoint.getAsDouble() + delta, PIVOT_MIN, PIVOT_MAX));
                break;
            case IDLE:
                m_idleSetpoint.set(
                    MathUtil.clamp(m_idleSetpoint.getAsDouble() + delta, PIVOT_MIN, PIVOT_MAX));
                break;
            case STORED:
                m_storedSetpoint.set(
                    MathUtil.clamp(m_storedSetpoint.getAsDouble() + delta, PIVOT_MIN, PIVOT_MAX));
                break;
        }

        logSetpoints();
    }

    
    private static double getActiveSetpointValue() {
        switch (getActiveSetpoint()) {
            case DEPLOYED: return m_deployedSetpoint.getAsDouble();
            case IDLE:     return m_idleSetpoint.getAsDouble();
            case STORED:   return m_storedSetpoint.getAsDouble();
            default:       return 0.0;
        }
    }

    private static void logSetpoints() {
        m_activeSetpointDisplay.set(getActiveSetpoint().name());
        Logger.recordOutput("IntakeConfig/Active", getActiveSetpoint().name());
        Logger.recordOutput("IntakeConfig/Deployed", m_deployedSetpoint.getAsDouble());
        Logger.recordOutput("IntakeConfig/Idle", m_idleSetpoint.getAsDouble());
        Logger.recordOutput("IntakeConfig/Stored", m_storedSetpoint.getAsDouble());
        Logger.recordOutput("IntakeConfig/Roller Speed", m_rollerSpeed.getAsDouble());
        Logger.recordOutput("IntakeConfig/Nudge Amount", m_nudgeAmount.getAsDouble());
    }
}
