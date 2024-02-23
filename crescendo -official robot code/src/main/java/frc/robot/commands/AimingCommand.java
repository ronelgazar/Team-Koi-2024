package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubSystem;

public class AimingCommand extends Command{
    private ShootingSubSystem shooter;
    private double angle;
    private long endTime;
     public AimingCommand(ShootingSubSystem shooter, double angle, long endTime) {
        this.shooter = shooter;
        this.angle = angle;
        endTime = System.currentTimeMillis();
     }

     public void initialize() {
        shooter.brakeAimer();
        shooter.brakeDelivery();
        shooter.brakeShooters();
     }

     public void execute() {
        shooter.setAimPosition(angle);
        try {
            TimeUnit.SECONDS.sleep((long) 0.5);
        } catch (InterruptedException e) {
            //notify user of me being an idiot
            e.printStackTrace();
        }
        initialize();
     }

     public void end() {
        initialize();
     }

     public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
     }
}
