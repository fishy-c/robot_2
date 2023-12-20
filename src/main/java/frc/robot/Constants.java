package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Conversions;

public final class Constants {
    public static final double stickDeadband = 0.4;

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        /* FRONT_LEFT */
        public static int FRONT_LEFT_DRIVE_ID = 4;
        public static int FRONT_LEFT_STEER_ID = 2;
        public static int FRONT_LEFT_CANCODER_ID = 12;
        public static Rotation2d FRONT_LEFT_CANCODER_OFFSET = Rotation2d.fromDegrees(178.418);
        public static InvertedValue FRONT_LEFT_DRIVE_INVERT = InvertedValue.CounterClockwise_Positive;
        public static InvertedValue FRONT_LEFT_STEER_INVERT = InvertedValue.Clockwise_Positive;
        public static SensorDirectionValue FRONT_LEFT_CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* BACK_LEFT */
        public static int BACK_LEFT_DRIVE_ID = 8;
        public static int BACK_LEFT_STEER_ID = 7;
        public static int BACK_LEFT_CANCODER_ID = 14;
        public static Rotation2d BACK_LEFT_CANCODER_OFFSET = Rotation2d.fromDegrees(204.17);
        public static InvertedValue BACK_LEFT_DRIVE_INVERT = InvertedValue.CounterClockwise_Positive;
        public static InvertedValue BACK_LEFT_STEER_INVERT = InvertedValue.Clockwise_Positive;
        public static SensorDirectionValue BACK_LEFT_CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* FRONT_RIGHT */
        public static int FRONT_RIGHT_DRIVE_ID = 5;
        public static int FRONT_RIGHT_STEER_ID = 9;
        public static int FRONT_RIGHT_CANCODER_ID = 13;
        public static Rotation2d FRONT_RIGHT_CANCODER_OFFSET = Rotation2d.fromDegrees(220.254);
        public static InvertedValue FRONT_RIGHT_DRIVE_INVERT = InvertedValue.Clockwise_Positive;
        public static InvertedValue FRONT_RIGHT_STEER_INVERT = InvertedValue.Clockwise_Positive;
        public static SensorDirectionValue FRONT_RIGHT_CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* BACK_RIGHT */
        public static int BACK_RIGHT_DRIVE_ID = 3;
        public static int BACK_RIGHT_STEER_ID = 1;
        public static int BACK_RIGHT_CANCODER_ID = 15;
        public static Rotation2d BACK_RIGHT_CANCODER_OFFSET = Rotation2d.fromDegrees(358.154);
        public static InvertedValue BACK_RIGHT_DRIVE_INVERT = InvertedValue.Clockwise_Positive;
        public static InvertedValue BACK_RIGHT_STEER_INVERT = InvertedValue.Clockwise_Positive;
        public static SensorDirectionValue BACK_RIGHT_CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;


        /* Drivetrain Constants */
        public static final double robotWidth = Units.inchesToMeters(23.25); //TODO: This must be tuned to specific robot
        public static final double robotLength = Units.inchesToMeters(23.25); //TODO: This must be tuned to specific robot
        public static final double wheelCircumferenceInMeters = 0.75206;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
  
        public static final Translation2d FL = new Translation2d(robotLength / 2.0, robotWidth / 2.0);
        public static final Translation2d FR = new Translation2d(robotLength / 2.0, -robotWidth / 2.0);
        public static final Translation2d BL = new Translation2d(-robotLength / 2.0, robotWidth / 2.0);
        public static final Translation2d BR = new Translation2d(-robotLength / 2.0, -robotWidth / 2.0);

        /* Module Gear Ratios */
        public static final double driveGearRatio = 6.55;
        public static final double angleGearRatio = 10.28;

        /* Angle Encoder Invert */
        
        /* Angle Motor PID Values */
        public static final double anglekP = 0.3;
        public static final double anglekI = 0;
        public static final double anglekD = 0;
        public static final double anglekS = 0;
        public static final double anglekV = 0;

        /* Drive Motor PID Values */
        public static final double drivekP = 0.3; //TODO: This must be tuned to specific robot
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;
        public static final double drivekS = 0;
        public static final double drivekV = 0;

        //public static final double driveKF = (1023/(Conversions.MPSToFalcon(5.0, Math.PI*Units.inchesToMeters(6), driveGearRatio)));

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.14643 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.8676 / 12);
        public static final double driveKA = (0.46219 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.0; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 3.0; //TODO: This must be tuned to specific robot

    }

    public static final class Intake{
        public static final double intakeVolts = 4;
    }

    public static final class Wrist{
        public static final double gearRatio = 25.92;
        public static final double rotorOffset0 =Conversions.DegreesToRotations(0, gearRatio); //wrist starts at bottom (for tuning purposes)
        public static final double rotorOffset1 =Conversions.DegreesToRotations(80, gearRatio); //wrist starts at top (idk)
        public static final double maxRangeInDegrees = 80;
        public static final double minRangeInDegrees = 0;
        public static final double maxRangeInRotations = Conversions.DegreesToRotations(112, gearRatio); //TODO: refind maxrange
        public static final double minRangeInRotations = Conversions.DegreesToRotations(0, gearRatio);

        public static final double kP = 0.8; //TODO: retune lol
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.05; 
        public static final double kV = 0; //kv =kf
        public static final double kG = 0;
       
        //prev: v=4000, a=400, no jerk, kP=1.5, kS=2 (in falconunits)
        public static final double CruiseVelocity = 6; 
        public static final double Acceleration = 12; 
        public static final double Jerk = 0; //

        public static final double homingOutput = 0.1;
        public static double customPos = Conversions.DegreesToRotations(45, gearRatio);

        //public static final double gravityFF = 0.04;

    }

    public static final class Elevator{
        public static final double gearRatio = 5;
        public static final double wheelCircumference = 0.159;
        public static final double maxHeightInMeters = 1.37;
        public static final double minHeightInMeters = 0;
        public static final double maxHeightInRotations = Conversions.metersToRotations(1.37, Constants.Elevator.wheelCircumference, Constants.Elevator.gearRatio);
        public static final double minHeightInRotations = Conversions.metersToRotations(0, Constants.Elevator.wheelCircumference, Constants.Elevator.gearRatio);

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;

        public static final double gravityFF = 0;

        public static final double CruiseVelocity = 7; //27000
        public static final double Acceleration = 14; //30000
        public static final double Jerk = 28; //idk

        public static final double testOutput = 0.3;
        public static final double homingOutput = -0.2;
        public static final double customHeight = Conversions.metersToRotations(1.37, Constants.Elevator.wheelCircumference, Constants.Elevator.gearRatio);

    }
}
