package frc.robot.utiils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

public class BrushedMotor extends CanSparkMotor {

    public BrushedMotor(int canID) {
        super(new CANSparkMax(canID, CANSparkLowLevel.MotorType.kBrushed));

    }

}