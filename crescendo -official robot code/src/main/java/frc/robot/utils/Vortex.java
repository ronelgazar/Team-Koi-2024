package frc.robot.utils;

public class Vortex extends RevMotor{
    public Vortex(int CANID, boolean isRelative){
        super(CANID, isRelative, false);
    }
}
