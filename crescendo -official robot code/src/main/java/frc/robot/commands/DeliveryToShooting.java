package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeliverySubSystem;

public class DeliveryToShooting extends Command{

    private final DeliverySubSystem m_DeliverySubSystem;
    /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public DeliveryToShooting(DeliverySubSystem subSystem){
        this.m_DeliverySubSystem = subSystem;
    }

    @Override
    public void execute(){
        m_DeliverySubSystem.setDeliveryVelocity(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
