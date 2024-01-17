package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PIDMotorController {
    private RelativeEncoder encoder;
    private CANSparkMax motor;
    private SparkPIDController pidController;
    private int deviceId;

    public PIDMotorController(int CANid)  {
        motor = new CANSparkMax(CANid, MotorType.kBrushless /*neo is brushed*/ );
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();
        deviceId = motor.getDeviceId();
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public CANSparkMax getMotor() {
        return motor;
    }

    public SparkPIDController getPidController() {
        return pidController;
    }

    public int getDeviceId() {
        return deviceId;
    }

    public void setPositionConversionFactor(double factor) {
        encoder.setVelocityConversionFactor(factor);
    }

    public void setVelocityConversionFactor(double factor) {
        encoder.setVelocityConversionFactor(factor);
    }

    public void setPIDConstants(PIDConstantsTempName pid) {
        pidController.setP(pid.getP());
        pidController.setI(pid.getI());
        pidController.setD(pid.getD());
        pidController.setFF(pid.getFF());
        pidController.setIZone(pid.getIZone());
    }

    public void setSetPoint(double angle, ControlType type) {
        pidController.setReference(angle, type);
    }
    
}