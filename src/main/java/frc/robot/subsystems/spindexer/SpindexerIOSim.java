package frc.robot.subsystems.spindexer;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SpindexerIOSim extends SpindexerIOHardware {
    private static final double RPM_TO_RAD_S = 2.0 * Math.PI / 60.0;

    private final SparkFlexSim m_motorSim = new SparkFlexSim(m_motor, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_physicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            SpindexerConstants.kSpindexerFeedforwardV / RPM_TO_RAD_S, 
            SpindexerConstants.kSpindexerFeedforwardA / RPM_TO_RAD_S
        ), 
        DCMotor.getNeoVortex(1)
    );

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {

        double voltage = m_motorSim.getAppliedOutput() * m_motorSim.getBusVoltage();
        m_physicsSim.setInputVoltage(voltage - Math.copySign(SpindexerConstants.kSpindexerFeedforwardS, voltage));
        m_physicsSim.update(0.02);

        m_motorSim.iterate(
            m_physicsSim.getAngularVelocityRPM(), 
            RobotController.getBatteryVoltage(), 
            0.02
        );

        super.updateInputs(inputs);
    }
}
