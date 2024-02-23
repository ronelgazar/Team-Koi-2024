package frc.robot.utiils;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utils.CanSparkMotor;

public class SparkFlexMotor extends CanSparkMotor {
    public SparkFlexMotor(int canID) {
        super(new CANSparkFlex(canID, MotorType.kBrushless));
    }

    
}