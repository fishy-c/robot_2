package frc.robot.commands.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Intake;

public class RunIntake extends CommandBase{
    private final Intake Intake;
    private boolean intakeOutake;

    public RunIntake(Intake Intake, boolean intakeOutake){
        System.out.printf("INTAKE CONSTRUCTOR");
        this.Intake = Intake;
        this.intakeOutake = intakeOutake;
        addRequirements(Intake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double output = Constants.Intake.intakeVolts;
        if(intakeOutake){
            Intake.setIntakeOutake(true);
        }
        else{
            Intake.setIntakeOutake(false);
            output *= -1;
        }
        Intake.intakeSpin(output);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        Intake.intakeSpin(0);
    }


    
}
