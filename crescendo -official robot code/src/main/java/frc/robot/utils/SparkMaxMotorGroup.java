package frc.robot.utils;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.revrobotics.SparkPIDController;

public class SparkMaxMotorGroup {
    ArrayList<CANSparkMax> motors;
    ArrayList<RelativeEncoder> encoders;
    ArrayList<SparkPIDController> pidControllers;
    ArrayList<Integer> ids;

    public SparkMaxMotorGroup(int... ids) {
        motors = new ArrayList<>();
        for (int i = 0; i < ids.length; i++) {
            CANSparkMax tempMotor = new CANSparkMax(ids[i], MotorType.kBrushless);
            setPidConstants();
            motors.add(tempMotor);
        }
    }

    public void setPidConstants() {
        for (int i = 0; i < motors.size(); i++) {
            encoders.get(i).setPositionConversionFactor(Constants.MotorAttributes.Neo.CONVERSION_FACTOR);
            pidControllers.get(i).setP(Constants.PIDConstants.P);
            pidControllers.get(i).setI(Constants.PIDConstants.I);
            pidControllers.get(i).setD(Constants.PIDConstants.D);
            pidControllers.get(i).setIZone(Constants.PIDConstants.IZONE);
            pidControllers.get(i).setFF(Constants.PIDConstants.FF);
            pidControllers.get(i).setOutputRange(Constants.MotorAttributes.Neo.MIN_OUTPUT_ENCODER,
                    Constants.MotorAttributes.Neo.MAX_OUTPUT_ENCODER);
        }
    }

    public MotorController[] getControllers() {
        MotorController[] res = new MotorController[motors.size()];
        for (int i = 0; i < res.length; i++) {
            res[i] = motors.get(i);
        }
        return res;
    }

    public void driveForward(double meters) {
        setSetPoint(meters);
    }

    public void setDrivePercent(double percent) {
        fo
    }

    public void setSetPoint(double setPoint) {
        for (int i = 0; i < motors.size(); i++) {
            encoders.get(i).setPosition(0);
            pidControllers.get(i).setReference(setPoint, ControlType.kPosition);
        }
    }
}
