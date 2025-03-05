package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class Homing extends Command{
    
    Elevator elevator;
    public Homing (Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
        this.elevator.homeElevatorDown();
    }

    @Override
    public void execute() {
        //this.elevator.getMotorE().set(-0.3);
    }
    
   @Override
   public void end(boolean interrupted) {
        this.elevator.resetEncoder();
   }

   @Override
    public boolean isFinished() {
        return this.elevator.isREVLimit();
    }
}
