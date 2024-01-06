package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase{
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    public final ModuleIO[] moduleIOs = new ModuleIO[4];
    private final ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };

    private Pose2d poseRaw = new Pose2d();
    private Rotation2d lastGyroYaw = new Rotation2d();
    private final boolean fieldRelatve;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.Swerve.FL, Constants.Swerve.FR, Constants.Swerve.BL,
        Constants.Swerve.BR);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(),
        getSwerveModulePositions()); 
    SwerveModuleState setpointModuleStates[] = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            0,
            new Rotation2d()));
    
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };

    public Swerve() {
        gyroIO = new GyroIOPigeon2(0);

        moduleIOs[0] = new ModuleIOTalonFX(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_STEER_ID, Constants.Swerve.FRONT_LEFT_CANCODER_ID, Constants.Swerve.FRONT_LEFT_CANCODER_OFFSET,
        Constants.Swerve.FRONT_LEFT_DRIVE_INVERT, Constants.Swerve.FRONT_LEFT_STEER_INVERT, Constants.Swerve.FRONT_LEFT_CANCODER_INVERT, "FRONT_LEFT");

        moduleIOs[1] = new ModuleIOTalonFX(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_STEER_ID, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, Constants.Swerve.FRONT_RIGHT_CANCODER_OFFSET,
        Constants.Swerve.FRONT_RIGHT_DRIVE_INVERT, Constants.Swerve.FRONT_RIGHT_STEER_INVERT, Constants.Swerve.FRONT_RIGHT_CANCODER_INVERT, "FRONT_RIGHT");

        moduleIOs[2] = new ModuleIOTalonFX(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_STEER_ID, Constants.Swerve.BACK_LEFT_CANCODER_ID, Constants.Swerve.BACK_LEFT_CANCODER_OFFSET,
        Constants.Swerve.BACK_LEFT_DRIVE_INVERT, Constants.Swerve.BACK_LEFT_STEER_INVERT, Constants.Swerve.BACK_LEFT_CANCODER_INVERT, "BACK_LEFT");

        moduleIOs[3] = new ModuleIOTalonFX(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_STEER_ID, Constants.Swerve.BACK_RIGHT_CANCODER_ID, Constants.Swerve.BACK_RIGHT_CANCODER_OFFSET,
        Constants.Swerve.BACK_RIGHT_DRIVE_INVERT, Constants.Swerve.BACK_RIGHT_STEER_INVERT, Constants.Swerve.BACK_RIGHT_CANCODER_INVERT, "BACK_RIGHT");
       
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].setDriveBrakeMode(true);
            moduleIOs[i].setTurnBrakeMode(false);
        }
        this.fieldRelatve = true;
      }


    @Override
    public void periodic(){
        updateOdometry();
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Swerve/Gyro", gyroInputs);
        for (int i = 0; i < 4; i++){
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Swerve/Module/ModuleNum[" + i + "]", moduleInputs[i]);
            moduleIOs[i].updateTunableNumbers();
        }

        logModuleStates("SwerveModuleStates/setpointStates", getSetpointStates());
        //logModuleStates("SwerveModuleStates/optimizedSetpointStates", getOptimizedSetPointStates());
        logModuleStates("SwerveModuleStates/MeasuredStates", getMeasuredStates());
        Logger.getInstance().recordOutput("Odometry/PoseRaw", poseRaw);
        
    }

    /* Tuning is the worst thing ive done in my life */
    public void requestVoltage(boolean testVoltage){
        if(testVoltage){
            for(int i = 0; i < 4; i++){
                moduleIOs[i].testDriveVoltage();
            }
         }
        else{
            for(int i = 0; i < 4; i++){
                moduleIOs[i].setDriveVoltage(0);
            }
        }
    }

    public void requestPercent(double x_speed, double y_speed,double rot_speed, boolean fieldRelative){

        Rotation2d[] steerPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            steerPositions[i] = new Rotation2d(moduleInputs[i].moduleAngleRads);
        }
        Rotation2d gyroPositionDegrees = new Rotation2d(gyroInputs.positionRad);
        if (fieldRelative){
            setpointModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed,
                y_speed,
                rot_speed,
                gyroPositionDegrees));
            kinematics.desaturateWheelSpeeds(setpointModuleStates, Constants.Swerve.maxSpeed);

            for (int i = 0; i < 4; i++) {
                moduleIOs[i].setDesiredState(setpointModuleStates[i]);
            }
        }
    }

    public void requestVelocity(double velocityMetersPerSecond, boolean auto){
        for (int i = 0 ; i < 4; i++){
            moduleIOs[i].setDriveVelocity(velocityMetersPerSecond, auto);
        }
    }

    public void zeroWheels(){
        for(int i = 0; i < 4; i++){
            moduleIOs[i].resetToAbsolute();
        }
    }

    public void zeroGyro(){
        gyroIO.reset();
    }

    public void updateOdometry(){
        var gyroYaw = new Rotation2d(gyroInputs.positionRad);
        lastGyroYaw = gyroYaw;
        poseRaw = odometry.update(
                getRotation2d(),
                getSwerveModulePositions());
    }

    public Pose2d getPoseRaw(){
        return poseRaw;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(gyroInputs.positionRad);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(
                    moduleInputs[i].driveDistanceMeters,
                    new Rotation2d(moduleInputs[i].moduleAngleRads));
        }
        return modulePositions;
    }

    public SwerveModuleState[] getMeasuredStates(){
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++){
            measuredStates[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, new Rotation2d(moduleInputs[i].moduleAngleRads));
        }
        return measuredStates;
    }
    public SwerveModuleState[] getSetpointStates(){
        return setpointModuleStates;
    }

    private void logModuleStates(String key, SwerveModuleState[] states) {
        List<Double> dataArray = new ArrayList<Double>();
        for (int i = 0; i < 4; i++) {
            dataArray.add(states[i].angle.getRadians());
            dataArray.add(states[i].speedMetersPerSecond);
        }
        Logger.getInstance().recordOutput(key, dataArray.stream().mapToDouble(Double::doubleValue).toArray());
    }

}