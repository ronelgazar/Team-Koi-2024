package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubSystem;

public class PushCommand extends Command{
    private final ClimbingSubSystem m_ClimbingSubSystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PushCommand(ClimbingSubSystem subSystem) {
    this.m_ClimbingSubSystem = subSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimbingSubSystem.push();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_ClimbingSubSystem.isCollapsed()){
      m_ClimbingSubSystem.setCollapsed(true);
    }
    return false;
  }
}
