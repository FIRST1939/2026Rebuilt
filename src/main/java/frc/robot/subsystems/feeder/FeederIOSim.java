package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim extends FeederIOHardware {

    private final SparkFlexSim m_motorSim = new SparkFlexSim(m_motor, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_physicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            Volts.of(FeederConstants.kFeederFeedforwardV).per(RPM).in(VoltsPerRadianPerSecond),
            Volts.of(FeederConstants.kFeederFeedforwardA).per(RPM.per(Second)).in(VoltsPerRadianPerSecondSquared)
        ),
        DCMotor.getNeoVortex(1)
    );

    @Override
    public void updateInputs(FeederIOInputs inputs) {

        double voltage = m_motorSim.getAppliedOutput() * m_motorSim.getBusVoltage();
        boolean overcomeFriction = Math.abs(voltage) > FeederConstants.kFeederFeedforwardS;
        m_physicsSim.setInputVoltage(overcomeFriction ? voltage - Math.copySign(FeederConstants.kFeederFeedforwardS, voltage) : 0);
        m_physicsSim.update(0.02);

        m_motorSim.iterate(
            m_physicsSim.getAngularVelocityRPM(), 
            RobotController.getBatteryVoltage(), 
            0.02
        );

        super.updateInputs(inputs);
    }
}
