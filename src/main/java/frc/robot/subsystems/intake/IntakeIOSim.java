package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim extends IntakeIOHardware {

    private static final double RPM_TO_RAD_S = 2.0 * Math.PI / 60.0;

    private final SparkFlexSim m_rollerMotorSim = new SparkFlexSim(m_rollerMotor, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_rollerPhysicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            IntakeConstants.kRollerFeedforwardV / RPM_TO_RAD_S,
            IntakeConstants.kRollerFeedforwardA / RPM_TO_RAD_S
        ), 
        DCMotor.getNeoVortex(1)
    );

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        double voltage = m_rollerMotorSim.getAppliedOutput() * m_rollerMotorSim.getBusVoltage();
        m_rollerPhysicsSim.setInputVoltage(voltage - Math.copySign(IntakeConstants.kRollerFeedforwardS, voltage));
        m_rollerPhysicsSim.update(0.02);

        m_rollerMotorSim.iterate(
            m_rollerPhysicsSim.getAngularVelocityRPM(), 
            RobotController.getBatteryVoltage(), 
            0.02
        );

        super.updateInputs(inputs);
    }
}
