package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrajectoryFollowerCommand extends CommandBase{
    private final Swerve swerve;
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Rotation2d> startHeading;

    public TrajectoryFollowerCommand(PathPlannerTrajectory trajectory, Supplier<Rotation2d> startHeading, Swerve swerve){
        this.trajectory = trajectory;
        this.startHeading = startHeading;
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        
    }

    
    
}
