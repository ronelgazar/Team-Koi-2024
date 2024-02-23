package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubSystem;

public class AngleControlCommand extends Command{
    private final ShootingSubSystem m_ShootingSubSystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AngleControlCommand(ShootingSubSystem subSys) {
    this.m_ShootingSubSystem = subSys;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShootingSubSystem.setAimPosition(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
