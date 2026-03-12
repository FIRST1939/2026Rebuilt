package frc.robot.subsystems.feeder;

import java.util.Random;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim extends FeederIOHardware {

    private static final double RPM_TO_RAD_S = 2.0 * Math.PI / 60.0;

    // Shot simulation constants
    private static final double SPIKE_MIN_INTERVAL = 0.1;  // seconds
    private static final double SPIKE_MAX_INTERVAL = 1;  // seconds
    private static final double SPIKE_CURRENT = 25.0;       // amps (above the 15A shot threshold)
    private static final double SPIKE_DURATION = 0.04;       // seconds (2 cycles)
    private static final double FEEDER_RUNNING_THRESHOLD = 100.0; // RPM

    private final SparkFlexSim m_motorSim = new SparkFlexSim(m_motor, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_physicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            FeederConstants.kFeederFeedforwardV / RPM_TO_RAD_S,
            FeederConstants.kFeederFeedforwardA / RPM_TO_RAD_S
        ),
        DCMotor.getNeoVortex(1)
    );

    // Shot simulation state
    private final Random m_random = new Random();
    private double m_nextSpikeCountdown = 0.0;
    private double m_spikeRemaining = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {

        double voltage = m_motorSim.getAppliedOutput() * m_motorSim.getBusVoltage();
        m_physicsSim.setInputVoltage(voltage - Math.copySign(FeederConstants.kFeederFeedforwardS, voltage));
        m_physicsSim.update(0.02);

        m_motorSim.iterate(
            m_physicsSim.getAngularVelocityRPM(), 
            RobotController.getBatteryVoltage(), 
            0.02
        );

        super.updateInputs(inputs);

        // Simulate current spikes while the feeder is running
        boolean feederRunning = Math.abs(inputs.feederVelocity) > FEEDER_RUNNING_THRESHOLD;

        if (feederRunning) {

            if (m_spikeRemaining > 0.0) {

                inputs.feederCurrent = SPIKE_CURRENT;
                m_spikeRemaining -= 0.02;
            } else {

                m_nextSpikeCountdown -= 0.02;

                if (m_nextSpikeCountdown <= 0.0) {

                    m_spikeRemaining = SPIKE_DURATION;
                    m_nextSpikeCountdown = SPIKE_MIN_INTERVAL 
                        + m_random.nextDouble() * (SPIKE_MAX_INTERVAL - SPIKE_MIN_INTERVAL);
                    inputs.feederCurrent = SPIKE_CURRENT;
                }
            }
        } else {

            // Reset when feeder stops
            m_nextSpikeCountdown = 0.0;
            m_spikeRemaining = 0.0;
        }
    }
}
