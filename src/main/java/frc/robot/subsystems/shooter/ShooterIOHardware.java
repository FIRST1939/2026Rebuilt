package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOHardware implements ShooterIO {
    
    protected final SparkFlex m_flywheelOne = new SparkFlex(ShooterConstants.flywheelOneCan, MotorType.kBrushless);

    protected final SparkFlex m_flywheelTwo = new SparkFlex(ShooterConstants.flywheelTwoCan, MotorType.kBrushless);

    protected final SparkFlex m_hood = new SparkFlex(ShooterConstants.hoodCan, MotorType.kBrushless);
}
