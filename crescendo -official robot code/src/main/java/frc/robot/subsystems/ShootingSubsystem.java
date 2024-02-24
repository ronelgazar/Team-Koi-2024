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

    public void setShootersIdleMode(IdleMode mode) {
        if(mode != IdleMode.kBrake) mode = IdleMode.kCoast;
        shootingMotorFront.setIdleMode(mode);
        shootingMotorBack.setIdleMode(mode);
    }
    public void setAimerIdleMode(IdleMode mode) {
        if(mode != IdleMode.kBrake) mode = IdleMode.kCoast;
        aimingMotor.setIdleMode(mode);
    }

    public void setDeliveryIdleMode(IdleMode mode) {
        if(mode != IdleMode.kBrake) mode = IdleMode.kCoast;
        shootingMotorFront.setIdleMode(mode);
    }

    public void brakeShooters() {
        setShootersIdleMode(IdleMode.kBrake);
        shootingMotorBack.set(0);
        shootingMotorFront.set(0);
    }

    public void brakeAimer() {
        setAimerIdleMode(IdleMode.kBrake);
        aimingMotor.set(0);
    }

    public void brakeDelivery() {
        setDeliveryIdleMode(IdleMode.kBrake);
        deliveryToShootingMotor.set(0);
    }

    public void coastShooters() {
        setShootersIdleMode(IdleMode.kCoast);
        shootingMotorBack.set(0);
        shootingMotorFront.set(0);
    }

    public void coastAimer() {
        setAimerIdleMode(IdleMode.kCoast);
        aimingMotor.set(0);
    }

    public void coastDelivery() {
        setDeliveryIdleMode(IdleMode.kCoast);
        deliveryToShootingMotor.set(0);
    }
}