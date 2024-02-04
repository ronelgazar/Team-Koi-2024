package frc.robot.utils;

public class Neo extends RevMotor{
    public Neo(int CANID, boolean isRelative){
        super(CANID, isRelative, true);
    }
}