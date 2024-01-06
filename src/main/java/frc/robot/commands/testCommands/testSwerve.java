// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class testSwerve extends CommandBase {
  /** Creates a new testSwerve. */
  private Swerve s_Swerve;
  private boolean testClosedLoop;
  private boolean testOutput;
  private double velocityMetersPerSecond;

  public testSwerve(Swerve swerve, boolean testClosedLoop, boolean testOutput, double velocityMetersPerSecond) {
    this.s_Swerve = swerve;

    this.testOutput = testOutput;
    this.testClosedLoop = testClosedLoop;
    this.velocityMetersPerSecond = velocityMetersPerSecond;

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(testClosedLoop){
    s_Swerve.requestVelocity(velocityMetersPerSecond, testClosedLoop);
   }
   else{
      s_Swerve.requestVoltage(testOutput);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
