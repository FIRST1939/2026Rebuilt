package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.shooter.ShooterIO;

public class SpindexerIOHardware implements ShooterIO {

    protected final SparkFlex m_motor = new SparkFlex(SpindexerConstants.kSpindexerCAN, MotorType.kBrushless);
}
