package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController pidController;
    private int id;

    public IntakeSubsystem(int id, MotorType type) {
        this.id = id;
        motor = new CANSparkMax(id, type);
        motor.restoreFactoryDefaults();
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();
        encoder.setPosition(0);
        /*
         * in the prev code (gitlab 2023 repository) there was the line:
         * encode.setVelocityConversionFactor(/conversion factor from constants/)
         * no idea what this is
         */
    }

    public  SparkPIDController getPidController() {
        return pidController;
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public int getId() {
        return id;
    }

    public CANSparkMax getController() {
        return motor; // ????? how is the motor the controller
    }

    // public void setPidConstants(){
    // encoder.setPositionConversionFactor(Constants.MotorAttributes.Neo.CONVERSION_FACTOR);
    // pidController.setP(Constants.PIDConstants.P);
    // pidController.setI(Constants.PIDConstants.I);
    // pidController.setD(Constants.PIDConstants.D);
    // pidController.setIZone(Constants.PIDConstants.IZONE);
    // pidController.setFF(Constants.PIDConstants.FF);
    // pidController.setOutputRange(Constants.MotorAttributes.Neo.MIN_OUTPUT_ENCODER,
    // Constants.MotorAttributes.Neo.MAX_OUTPUT_ENCODER);
    // SmartDashboard.putNumber("P Gain", Constants.PIDConstants.P);
    // SmartDashboard.putNumber("I Gain", Constants.PIDConstants.I);
    // SmartDashboard.putNumber("D Gain", Constants.PIDConstants.D);
    // SmartDashboard.putNumber("I Zone",Constants.PIDConstants.IZONE);
    // SmartDashboard.putNumber("Feed Forward",Constants.PIDConstants.FF);
    // SmartDashboard.putNumber("Max Output",
    // Constants.MotorAttributes.Neo.MAX_OUTPUT_ENCODER);
    // SmartDashboard.putNumber("Min Output",
    // Constants.MotorAttributes.Neo.MIN_OUTPUT_ENCODER);

    // display Smart Motion coefficients
    // SmartDashboard.putNumber("Max Velocity",
    // Constants.SmartMotionConstants.MAX_VEL);
    // SmartDashboard.putNumber("Min Velocity",
    // Constants.SmartMotionConstants.MIN_VEL);
    // SmartDashboard.putNumber("Max Acceleration",
    // Constants.SmartMotionConstants.MAX_ACCEL);
    // SmartDashboard.putNumber("Allowed Closed Loop Error",
    // Constants.SmartMotionConstants.ALLOWED_ERROR);
    // SmartDashboard.putNumber("Set Position", 0);
    // SmartDashboard.putNumber("Set Velocity", 0);
    // this.pidController.setSmartMotionAllowedClosedLoopError(Constants.SmartMotionConstants.ALLOWED_ERROR,
    // 0);
    // this.pidController.setSmartMotionMaxAccel(Constants.SmartMotionConstants.MAX_ACCEL,
    // 0);
    // this.pidController.setSmartMotionMaxVelocity(Constants.SmartMotionConstants.MAX_VEL,
    // 0);
    // this.pidController.setSmartMotionMinOutputVelocity(10,0);
    // }

    public void input() {
        motor.set(0.5); // according to docs, -1 < speed < 1 so i put it at -0.5 to turn on the motors
                         // for intake to not make them spin too fast
        try {
            TimeUnit.SECONDS.sleep(3); // i thought 3 seconds would be enough to get the disc in
        } catch (InterruptedException e) {
            // ????? idk what to do here
            // maybe throw something
        }

        motor.set(0);
        motor.setIdleMode(IdleMode.kBrake); // there may be a problem if a disc doesn't manage to get through in time
                                            // and gets stuck but
        // I didn't want the intake gears spinning around
    }

    public void turnOn(double speed) {
        motor.set(speed);
    }

    public void turnOff() {
        coast();
    }

    public void stop() {
        //motor.stopMotor();
        motor.set(0);
    }

    public void setSetPoint(double setPoint) {
        encoder.setPosition(0);
        pidController.setReference(setPoint, ControlType.kPosition);
    } //someone needs to explain this to me

    public void setIdleMode(IdleMode mode) {
        motor.setIdleMode(mode == IdleMode.kBrake ? IdleMode.kBrake : IdleMode.kCoast); //the if is there for if the user inputs something that is not break or coast (somehow)
    }

    public void brake() {
        motor.set(0);
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void coast() {
        // motor.setVoltage(0); // stop electricity feed to the motor???
        motor.set(0);
        motor.setIdleMode(IdleMode.kCoast);
    }
    
    public void periodic() {}
    public void simulationPeriodic() {}
}
