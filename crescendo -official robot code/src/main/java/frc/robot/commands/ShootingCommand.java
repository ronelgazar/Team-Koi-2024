package frc.robot.commands;
import frc.robot.subsystems.ShootingSubSystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootingCommand extends Command {
    private final ShootingSubSystem m_shooter;

    public ShootingCommand(ShootingSubSystem m_shooter) {
        this.m_shooter = m_shooter;
    }

    @Override
    public void execute() {
        m_shooter.setShootersVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
