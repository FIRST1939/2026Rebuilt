package frc.robot.bindings.characterization;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bindings.BindingParams;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCharacterizationBindings {
    
    private final LoggedNetworkNumber m_setpoint = new LoggedNetworkNumber("/Tuning/Intake/Pivot Setpoint", 0);

    private final LoggedNetworkNumber m_leftPivotP = new LoggedNetworkNumber("/Tuning/Intake/Left Pivot P", IntakeConstants.kLeftPivotFeedbackP);
    private final LoggedNetworkNumber m_leftPivotD = new LoggedNetworkNumber("/Tuning/Intake/Left Pivot D", IntakeConstants.kLeftPivotFeedbackD);

    private final LoggedNetworkNumber m_rightPivotP = new LoggedNetworkNumber("/Tuning/Intake/Right Pivot P", IntakeConstants.kRightPivotFeedbackP);
    private final LoggedNetworkNumber m_rightPivotD = new LoggedNetworkNumber("/Tuning/Intake/Right Pivot D", IntakeConstants.kRightPivotFeedbackD);

    private final LoggedNetworkNumber m_cruiseVelocity = new LoggedNetworkNumber("/Tuning/Intake/Pivot Cruise Velocity", 0);
    private final LoggedNetworkNumber m_maxAcceleration = new LoggedNetworkNumber("/Tuning/Intake/Pivot Max Acceleration", 0);
    private final LoggedNetworkNumber m_allowedError = new LoggedNetworkNumber("/Tuning/Intake/Pivot Allowed Error", 0);

    private DoubleSupplier m_leftPivotError;
    private DoubleSupplier m_rightPivotError;

    public IntakeCharacterizationBindings(BindingParams bindingParams, Trigger modeTrigger) {

        modeTrigger.and(bindingParams.operatorController.leftBumper()).whileTrue(bindingParams.intake.rollerSysIdQuasistaticForward());
        modeTrigger.and(bindingParams.operatorController.rightBumper()).whileTrue(bindingParams.intake.rollerSysIdQuasistaticReverse());
        modeTrigger.and(bindingParams.operatorController.leftTrigger()).whileTrue(bindingParams.intake.rollerSysIdDynamicForward());
        modeTrigger.and(bindingParams.operatorController.rightTrigger()).whileTrue(bindingParams.intake.rollerSysIdDynamicReverse());

        modeTrigger.and(bindingParams.operatorController.povUp()).whileTrue(bindingParams.intake.leftPivotSysIdQuasistaticForward());
        modeTrigger.and(bindingParams.operatorController.povRight()).whileTrue(bindingParams.intake.leftPivotSysIdQuasistaticReverse());
        modeTrigger.and(bindingParams.operatorController.povDown()).whileTrue(bindingParams.intake.leftPivotSysIdDynamicForward());
        modeTrigger.and(bindingParams.operatorController.povLeft()).whileTrue(bindingParams.intake.leftPivotSysIdDynamicReverse());

        modeTrigger.and(bindingParams.operatorController.y()).whileTrue(bindingParams.intake.rightPivotSysIdQuasistaticForward());
        modeTrigger.and(bindingParams.operatorController.b()).whileTrue(bindingParams.intake.rightPivotSysIdQuasistaticReverse());
        modeTrigger.and(bindingParams.operatorController.a()).whileTrue(bindingParams.intake.rightPivotSysIdDynamicForward());
        modeTrigger.and(bindingParams.operatorController.x()).whileTrue(bindingParams.intake.rightPivotSysIdDynamicReverse());

        modeTrigger.and(bindingParams.operatorController.rightStick()).onTrue(Commands.runOnce(() -> {

            bindingParams.intake.updateLeftPivotControllerFeedback(
                m_leftPivotP.getAsDouble(),
                m_leftPivotD.getAsDouble()
            );

            bindingParams.intake.updateRightPivotControllerFeedback(
                m_rightPivotP.getAsDouble(),
                m_rightPivotD.getAsDouble()
            );

            bindingParams.intake.updatePivotControllerProfile(
                m_cruiseVelocity.getAsDouble(),
                m_maxAcceleration.getAsDouble(),
                m_allowedError.getAsDouble()
            );

            double setpoint = m_setpoint.getAsDouble();
            m_leftPivotError = () -> setpoint - bindingParams.intake.getLeftPivotPosition();
            m_rightPivotError = () -> setpoint - bindingParams.intake.getRightPivotPosition();
            bindingParams.intake.setPivotPosition(setpoint);
        }, bindingParams.intake));

        modeTrigger.and(bindingParams.operatorController.rightStick()).onFalse(Commands.runOnce(() -> {

            bindingParams.intake.setPivotPercentage(0);
        }, bindingParams.intake));
    }

    @AutoLogOutput(key = "/Tuning/Intake/Left Pivot Error")
    private double getLeftPivotError() {

        return m_leftPivotError.getAsDouble();
    }

    @AutoLogOutput(key = "/Tuning/Intake/Right Pivot Error")
    private double getRightPivotError() {

        return m_rightPivotError.getAsDouble();
    }
}
