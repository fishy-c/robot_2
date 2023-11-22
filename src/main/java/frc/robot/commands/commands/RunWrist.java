package frc.robot.commands.commands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist.Wrist;


public class RunWrist extends CommandBase{
    private final Wrist Wrist;
    private final boolean Raise;

    public RunWrist(Wrist Wrist, boolean Raise){
        this.Wrist = Wrist;
        this.Raise = Raise;
        addRequirements(Wrist);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(Raise){
            Wrist.setSetPoint(Constants.Wrist.maxRangeInDegrees);
          }
        else{
            Wrist.setSetPoint(Constants.Wrist.minRangeInDegrees);
        }   
        Wrist.moveWrist();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        
    }


    
}

