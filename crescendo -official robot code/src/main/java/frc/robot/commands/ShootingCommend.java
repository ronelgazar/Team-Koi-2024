package frc.robot.commands;
import frc.robot.subsystems.ShootingSubSystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootingCommend extends Command {
    private final ShootingSubSystem m_shooter;

    public ShootingCommend(ShootingSubSystem m_shooter) {
        this.m_shooter = m_shooter;
    }
}
