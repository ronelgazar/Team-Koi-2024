package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utils.Neo;

public class ClimbingSubSystem {

    DigitalInput leftClimbingLimitSwitch = new DigitalInput(Constants.ClimbingSubSysConstants.LEFT_CLIMBING_LIMIT_SWITCH_CHANNER);

    private boolean collapsed = true;


    private Neo leftMotor, rightMotor;

    public ClimbingSubSystem(){
        this.leftMotor = new Neo(Constants.ClimbingSubSysConstants.LEFT_MOTOR_ID, true);
        this.rightMotor = new Neo(Constants.ClimbingSubSysConstants.RIGHT_MOTOR_ID, true);
        
        rightMotor.follow(leftMotor.getMotorController());
        rightMotor.setInverted(true);
    }
    /**
     * @param power When power is positive = Goes up. When power negetive = Goes down.
     */
    public void pull(){
        leftMotor.setPower(0.7);
        
    }

    public boolean isCollapsed()
    {
        return collapsed;
    }

    public void setCollapsed(boolean isCollpased)
    {
        collapsed = isCollpased;
    }


    public void push(){
        leftMotor.setPower(-0.7);
    }


    public void moveLimited(double power){

        if(!leftClimbingLimitSwitch.get()){
            leftMotor.setPower(power);
        }
    }

    public boolean getLimitSwitchState(){
        return leftClimbingLimitSwitch.get();
    }
}
