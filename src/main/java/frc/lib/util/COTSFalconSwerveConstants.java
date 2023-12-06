package frc.lib.util;

//import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class COTSFalconSwerveConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKF;
    public final boolean cancoderInvert;

    public COTSFalconSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP, double angleKI, double angleKD, double angleKF, boolean cancoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKF = angleKF;
        this.cancoderInvert = cancoderInvert;
    }
    /** Swerve Drive Specialties - SwerveX Module*/
    public static COTSFalconSwerveConstants SwerveX(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(6.0);
 
        /** 10.28 : 1 */
        double angleGearRatio = (10.28 / 1.0);
 
        double angleKP = 0.3;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;
        
        boolean cancoderInvert = false;
        return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, cancoderInvert);
    }
    
    // /** Swerve Drive Specialties - MK3 Module*/
    // public static COTSFalconSwerveConstants SDSMK3(double driveGearRatio){
    //     double wheelDiameter = Units.inchesToMeters(4.0);
 
    //     /** 12.8 : 1 */
    //     double angleGearRatio = (12.8 / 1.0);
 
    //     double angleKP = 0.2;
    //     double angleKI = 0.0;
    //     double angleKD = 0.0;
    //     double angleKF = 0.0;
 
    //     boolean driveMotorInvert = false;
    //     boolean angleMotorInvert = false;
    //     boolean canCoderInvert = false;
    //     return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
    // }

    // /** Swerve Drive Specialties - MK4 Module*/
    // public static COTSFalconSwerveConstants SDSMK4(double driveGearRatio){
    //     double wheelDiameter = Units.inchesToMeters(4.0);
 
    //     /** 12.8 : 1 */
    //     double angleGearRatio = (12.8 / 1.0);
 
    //     double angleKP = 0.2;
    //     double angleKI = 0.0;
    //     double angleKD = 0.0;
    //     double angleKF = 0.0;
 
    //     boolean driveMotorInvert = false;
    //     boolean angleMotorInvert = false;
    //     boolean canCoderInvert = false;
    //     return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
    // }

    // /** Swerve Drive Specialties - MK4i Module*/
    // public static COTSFalconSwerveConstants SDSMK4i(double driveGearRatio){
    //     double wheelDiameter = Units.inchesToMeters(4.0);

    //     /** (150 / 7) : 1 */
    //     double angleGearRatio = ((150.0 / 7.0) / 1.0);

    //     double angleKP = 0.3;
    //     double angleKI = 0.0;
    //     double angleKD = 0.0;
    //     double angleKF = 0.0;

    //     boolean driveMotorInvert = false;
    //     boolean angleMotorInvert = true;
    //     boolean canCoderInvert = false;
    //     return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
    // }

    /* Drive Gear Ratios for all supported modules */
    public class driveGearRatios{
        /* SwerveX */
        public static final double SwerveX = (6.55 / 1.0);
        /* SDS MK3 */
        /** SDS MK3 - 8.16 : 1 */
        public static final double SDSMK3_Standard = (8.16 / 1.0);
        /** SDS MK3 - 6.86 : 1 */
        public static final double SDSMK3_Fast = (6.86 / 1.0);

        /* SDS MK4 */
        /** SDS MK4 - 8.14 : 1 */
        public static final double SDSMK4_L1 = (8.14 / 1.0);
        /** SDS MK4 - 6.75 : 1 */
        public static final double SDSMK4_L2 = (6.75 / 1.0);
        /** SDS MK4 - 6.12 : 1 */
        public static final double SDSMK4_L3 = (6.12 / 1.0);
        /** SDS MK4 - 5.14 : 1 */
        public static final double SDSMK4_L4 = (5.14 / 1.0);
        
        /* SDS MK4i */
        /** SDS MK4i - 8.14 : 1 */
        public static final double SDSMK4i_L1 = (8.14 / 1.0);
        /** SDS MK4i - 6.75 : 1 */
        public static final double SDSMK4i_L2 = (6.75 / 1.0);
        /** SDS MK4i - 6.12 : 1 */
        public static final double SDSMK4i_L3 = (6.12 / 1.0);
    }
}

  