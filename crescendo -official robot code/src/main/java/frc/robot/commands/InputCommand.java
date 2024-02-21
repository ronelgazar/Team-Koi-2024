package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeliverySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//supposed to get input and turn on delivery to be ready for shooting
//MAIN METHOD IS ready() OR ready(int seconds)
public class InputCommand extends Command{
    private IntakeSubsystem intake;
    private DeliverySubsystem delivery;
    private int runTimeSeconds;
    private long endTime;

    public InputCommand(IntakeSubsystem i, DeliverySubsystem d) {
        intake = i;
        delivery = d;
        runTimeSeconds = 5;
        endTime = System.currentTimeMillis() + runTimeSeconds;
    }

    public InputCommand(IntakeSubsystem i, DeliverySubsystem d, int rts) {
        intake = i;
        delivery = d;
        runTimeSeconds = rts;
        endTime = System.currentTimeMillis() + runTimeSeconds;
    }

    /**
     * main method for getting input and feeding to the gun
     * if you want to control the time this turns on try the method: ready(int seconds)
     */
    public void ready() {
        intake.input(runTimeSeconds);
        delivery.deliver(runTimeSeconds);

        //for good measure
        intake.brake();
        delivery.brake();
    }

    /**
     * @param seconds : number of seconds you want to turn on the input
     * 
     * main method for getting input and feeding to the gun
     */
    public void ready(int seconds) {
        setRunTimeSeconds(seconds);
        ready();
    }

    public void init() {
        intake.brake();
        delivery.brake();
    }

    public void execute() {
        ready();
    }
    
    public void end(boolean interrupted) {
        init();
    }

    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }

    public void setRunTimeSeconds(int seconds) {
        if(!(seconds < 0 || seconds > 30))
            runTimeSeconds = seconds;
        else {
            // TODO log or display an error to the user without crashing the program
        }
    }

    public int getRunTimeSeconds() {
        return runTimeSeconds;
    }
}
