package frc.robot.commands.testCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Elevator;

public class testElevator extends CommandBase{
    private final Elevator Elevator;
    private final boolean testOutput;
    private final boolean testing;
    private double output;
    private double setPoint;

    public testElevator(Elevator Elevator, boolean testOutput, boolean testing, double output, double setPoint){
        this.Elevator = Elevator;
        this.testOutput = testOutput;
        this.output = output;
        this.setPoint = setPoint;
        this.testing = testing;

        addRequirements(Elevator);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(testing){
            Elevator.testOutput();
        }
        else if(testOutput){
            System.out.printf("EXECUTING ELEVATOR");
            Elevator.setOutput(output);
        }
        else{
            Elevator.setHeight(setPoint);
            Elevator.setElevator();
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

