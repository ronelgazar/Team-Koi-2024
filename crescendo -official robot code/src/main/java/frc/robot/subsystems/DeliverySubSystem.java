package frc.robot.subsystems;
import frc.robot.utils.SparkMaxMotor;
import frc.robot.Constants;

public class DeliverySubSystem {
    private SparkMaxMotor deliveryRightMotor;
    private SparkMaxMotor deliveryLeftMotor;

    public DeliverySubSystem(){

        deliveryRightMotor = new SparkMaxMotor(Constants.DeliveryMotorPortConstants.kDeliveryMotorRightPort);
        deliveryLeftMotor = new SparkMaxMotor(Constants.DeliveryMotorPortConstants.kDeliveryMotorLeftPort);
        deliveryLeftMotor.setInverted(true);
        deliveryLeftMotor.follow(deliveryLeftMotor);
    }

    public void setDeliveryVelocity(double Velocity){
        deliveryRightMotor.set(Velocity);
    }

}
