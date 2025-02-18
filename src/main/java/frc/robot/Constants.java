package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 13;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21); // Tuned
        public static final double wheelBase = Units.inchesToMeters(23); // Tuned
        public static final double wheelCircumference = 4 * Math.PI;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 265; // Tuned
        public static final double angleGearRatio = 1;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.Clockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 10;
        public static final int angleCurrentThreshold = 20;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 30;
        public static final int driveCurrentThreshold = 30;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 90;
        public static final double angleKI = 0.001;
        public static final double angleKD = 0.1;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.09; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // Tuned
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 8; // TODO: This must be tuned to specific robot
        public static final double maxAutoSpeed = 2.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = Math.PI * 3; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod1 { // Tuned
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 2;
            public static final int configNum = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); // 76.49
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, configNum);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod0 { // Tuned
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 5;
            public static final int configNum = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); // -177.08
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, configNum);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // Tuned
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 8;
            public static final int configNum = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); // -5.69
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, configNum);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // Tuned
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 11;
            public static final int configNum = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0); // -122.58
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, configNum);
        }
    }

    public static final class AutoConstants { // Tuned
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static double targetPositionTolerance = 0.1;
    public static double targetAngleTolerance = 3;
    public static final double Source_Distance = 15.2;
    public static final double L2_Distance = 19;
    public static final double L3_Distance = 34;
    public static final double L4_Distance = 71;
    public static final double resetPos = 0;
    public static final double L4_Angle = 14;
    public static final double L3_Angle = 11;
    public static final double Source_Angle = 7.2;
    public static final double Rest_Angle = 0;
}