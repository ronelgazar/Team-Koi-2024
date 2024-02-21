package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeliverySubsystem extends SubsystemBase {
    private CANSparkMax motor1, motor2; // 2 Neo motors for delivery (hovala)
    private RelativeEncoder encoder1, encoder2;
    private SparkPIDController pidController1, pidController2;

    public DeliverySubsystem() {
        motor1 = new CANSparkMax(Constants.Id.Motors.Delivery.MOTOR1, CANSparkLowLevel.MotorType.kBrushless); //i think this is a neo motor
        motor2 = new CANSparkMax(Constants.Id.Motors.Delivery.MOTOR2, CANSparkLowLevel.MotorType.kBrushless);
        //motor2.setInverted(true);

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();
        
        pidController1 = motor1.getPIDController();
        pidController2 = motor2.getPIDController();
    }

    public void deliver(int time) {
        turnOn(0.5);
        try {
            TimeUnit.SECONDS.sleep(time);
        } catch (InterruptedException e) {}
        brake();
    }

    public void deliver() {
        deliver(3);
    }

    public void turnOn(double speed) {
        if(!(speed > 1 || speed < -1)) {
            motor1.set(speed);
            motor2.set(speed);
        }
    }

    public void turnOff() {
        coast();
    }

    public void stop() {
        // motor.stopMotor();
        motor1.set(0);
        motor2.set(0);
    }

    public void setSetPoint(double setPoint) {
        encoder1.setPosition(0);
        encoder2.setPosition(0);
        pidController1.setReference(setPoint, ControlType.kPosition);
        pidController2.setReference(setPoint, ControlType.kPosition);
    } // someone needs to explain this to me

    public void setIdleMode(IdleMode mode) {
        mode = mode == IdleMode.kBrake ? IdleMode.kBrake : IdleMode.kCoast;
        motor1.setIdleMode(mode);
        motor2.setIdleMode(mode);
    }

    public void brake() {
        motor1.set(0);
        motor2.set(0);
        motor1.setIdleMode(IdleMode.kBrake);
        motor2.setIdleMode(IdleMode.kBrake);
    }

    public void coast() {
        // motor.setVoltage(0); // stop electricity feed to the motor???
        motor1.set(0);
        motor2.set(0);
        motor1.setIdleMode(IdleMode.kCoast);
        motor2.setIdleMode(IdleMode.kCoast);
    }

    public void periodic() {
    }

    public void simulationPeriodic() {
    }
}
