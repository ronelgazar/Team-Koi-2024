package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubSystem;

public class ShootingCommand extends Command{
    private ShootingSubSystem shooter;
    private long endTime;
    private int runTime;
    private double shootingVelocity;

    public ShootingCommand(ShootingSubSystem shooter, double shootingVelocity, int runTime) {
        this.shooter = shooter;
        this.runTime = runTime;
        this.shootingVelocity = shootingVelocity;
        endTime = System.currentTimeMillis();
    }

    public void initialize() {
        shooter.brakeAimer();
        shooter.brakeDelivery();
        shooter.brakeShooters();
    }

    public void execute() {
        shooter.setDeliveryVelocity(0.2);//arbitrary value
        shooter.setShootersVelocity(shootingVelocity);
        //EXPLANATION:
        //i turn on the delivery motor to bring the disc.
        //while the disc makes its way to the shooter i turn on the shooter motor
        //as soon as the disc makes it to the shooter it launches at the specified speed (default=0.6)
        try {
            TimeUnit.SECONDS.sleep(runTime);
            //I think 1 second is enough for the disc to be shot but not enough
            //to mess up the next disc
        } catch (InterruptedException e) {
            //im assuming we end up here if the .sleep input is bad
            //but idk when this will get thrown
            e.printStackTrace(); //log the error
        }
        initialize();
    }

    public void end(boolean interrupted) {
        initialize();
    }

    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }

    public void setRunTime(int seconds) {
        if(!(seconds < 0 || seconds > 30))
            runTime = seconds;
        else {
            //idk
        }
    }
}
