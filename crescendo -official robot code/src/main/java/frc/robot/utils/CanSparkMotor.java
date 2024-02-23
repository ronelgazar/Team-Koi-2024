package frc.robot.utils;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;


public abstract class CanSparkMotor {

    private CANSparkBase motorController;
    private SparkPIDController pidController;
    private RelativeEncoder encoder;

    public CanSparkMotor(CANSparkBase motorController) {
        this.motorController = motorController;
        this.pidController = motorController.getPIDController();
        this.encoder = motorController.getEncoder();
    }

    public void set(double speed) {
        motorController.set(speed);
    }

    public void setInverted(boolean isInverted) {
        motorController.setInverted(isInverted);
    }

    public void follow(CanSparkMotor leader) {
        motorController.follow(leader.motorController);
    }

    public CANSparkBase getMotorController(){
        return motorController;
    }

    public void setPIDConstants(PIDConstants pidConstants) {
        pidController.setP(pidConstants.kP());
        pidController.setI(pidConstants.kI());
        pidController.setD(pidConstants.kD());
        pidController.setFF(pidConstants.kF());
        pidController.setIZone(pidConstants.kIZ());

    }

    public void setSetPoint(double setPoint, ControlType controlType) {
        pidController.setReference(setPoint, controlType);

    }

    public void setFeedbackDevice(RelativeEncoder encoder) {

        pidController.setFeedbackDevice(encoder);
    }

    public void setFeedbackDevice(AbsoluteEncoder encoder) {
        pidController.setFeedbackDevice(encoder);
    }

}
