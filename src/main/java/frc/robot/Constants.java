package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.math.Conversions;

public final class Constants {
    public static final double stickDeadband = 0.05;

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SwerveX(COTSFalconSwerveConstants.driveGearRatios.SwerveX);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.25); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23.25); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        // /* Motor Inverts */
        // public static final TalonFXInvertType angleMotorInvert = chosenModule.angleMotorInvert;
        // public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.3; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = (1023/(Conversions.MPSToFalcon(5.0, Math.PI*Units.inchesToMeters(6), driveGearRatio)));

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.14643 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.8676 / 12);
        public static final double driveKA = (0.46219 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 7.0; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot (leftFront)
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(178.418);
            public static final boolean driveMotorInvert = true;
            public static final TalonFXInvertType angleMotorInvert = TalonFXInvertType.Clockwise;            
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert, angleMotorInvert);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot (rightFront)
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(220.254);
            public static final boolean driveMotorInvert = false;
            public static final TalonFXInvertType angleMotorInvert = TalonFXInvertType.Clockwise;    
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert, angleMotorInvert);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot (leftBack)
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(204.17);
            public static final boolean driveMotorInvert = true;
            public static final TalonFXInvertType angleMotorInvert = TalonFXInvertType.Clockwise;    
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert, angleMotorInvert);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot (rightBack)
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(358.154);
            public static final boolean driveMotorInvert = false;
            public static final TalonFXInvertType angleMotorInvert = TalonFXInvertType.Clockwise;    
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert, angleMotorInvert);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
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
        public static final double maxRangeInRotations = Conversions.DegreesToRotations(80, gearRatio); //TODO: refind maxrange
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
