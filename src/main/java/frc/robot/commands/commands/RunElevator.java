
package frc.robot.commands.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.Elevator;

public class RunElevator extends CommandBase{
    private final Elevator Elevator;
    private final boolean raise;

    public RunElevator(Elevator Elevator, boolean raise){
        this.Elevator = Elevator;
        this.raise = raise;
        addRequirements(Elevator);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(raise){
            Elevator.setHeight(Constants.Elevator.maxHeightInMeters);
        }
        else{
           Elevator.setHeight(Constants.Elevator.minHeightInMeters);
        }
        Elevator.setElevator();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        
    }


    
}
