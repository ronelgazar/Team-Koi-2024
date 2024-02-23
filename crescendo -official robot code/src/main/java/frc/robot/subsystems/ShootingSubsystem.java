package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utiils.BrushedMotor;
import frc.robot.utiils.SparkFlexMotor;
import frc.robot.utiils.SparkMaxMotor;

public class ShootingSubSystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    private SparkFlexMotor shootingMotorFront;
    private SparkFlexMotor shootingMotorBack;
    private SparkMaxMotor aimingMotor;
    private BrushedMotor deliveryToShootingMotor;

    public ShootingSubSystem() {
        shootingMotorFront = new SparkFlexMotor(Constants.Id.Motors.Shooter.SHOOTER_MOTOR_1);
        shootingMotorBack = new SparkFlexMotor(Constants.Id.Motors.Shooter.SHOOTER_MOTOR_2);
        shootingMotorBack.follow(shootingMotorFront);
        aimingMotor = new SparkMaxMotor(Constants.Id.Motors.Shooter.AIMING_MOTOR);
        deliveryToShootingMotor = new BrushedMotor(Constants.Id.Motors.Shooter.FEED_MOTOR);
    }

    public void setShootersVelocity(double Velocity) {
        shootingMotorFront.set(Velocity);
    }

    public void setAimPosition(double Position) {
        aimingMotor.set(Position);
    }

    public void setDeliveryVelocity(double Velocity) {
        deliveryToShootingMotor.set(Velocity);
    }

    public void setShooterIdleMode(IdleMode mode) {
        shootingMotorFront.setIdleMode(mode == IdleMode.kBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }

}