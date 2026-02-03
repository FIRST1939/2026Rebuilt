package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeIOHardware implements IntakeIO {
    
    protected final SparkFlex m_roller = new SparkFlex(IntakeConstants.kRollerCAN, MotorType.kBrushless);
    protected final SparkFlex m_pivotOne = new SparkFlex(IntakeConstants.kPivotFollowerCAN, MotorType.kBrushless);
    protected final SparkFlex m_pivotTwo = new SparkFlex(IntakeConstants.kPivotFollowerCAN, MotorType.kBrushless);
}
