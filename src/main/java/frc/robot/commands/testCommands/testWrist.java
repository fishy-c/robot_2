package frc.robot.commands.testCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist.Wrist;


public class testWrist extends CommandBase{
    private final Wrist Wrist;
    private final boolean testOutput;
    private final double output;
    private double setPoint;

 

    public testWrist(Wrist Wrist, boolean testOutput, double output, double setPoint){
        this.Wrist = Wrist;
        this.testOutput = testOutput;
        this.output = output;
        this.setPoint = setPoint;
        addRequirements(Wrist);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(testOutput){
            Wrist.setOutput(output);
        }
        else{
            Wrist.setSetPoint(setPoint);
            Wrist.moveWrist();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        
    }


    
}


