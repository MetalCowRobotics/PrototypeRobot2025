package frc.robot.subsystems;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


public final class config {
    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction =
            (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps =
            (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
      }
    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
      }
    public static final class Configs {
        public static final class MAXSwerveModule {
          public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
          public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

      
          static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor =
                ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
      
            drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
            drivingConfig
                .encoder
                .positionConversionFactor(drivingFactor) // meters
                .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(0.04, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);
      
            turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
            turningConfig
                .absoluteEncoder
                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering motor in the MAXSwerve Module.
                .inverted(true)
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
          }
        }
      
        public static final class CoralSubsystem {
          public static final SparkMaxConfig armConfig = new SparkMaxConfig();
          public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
          public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
      
          static {
            // Configure basic settings of the arm motor
            armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
      
            /*
             * Configure the closed loop controller. We want to make sure we set the
             * feedback sensor as the primary encoder.
             */
            armConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(0.25);
      
            // Configure basic settings of the elevator motor
            elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
      
            /*
             * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
             * will prevent any actuation of the elevator in the reverse direction if the limit switch is
             * pressed.
             */
            elevatorConfig
                .limitSwitch
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(Type.kNormallyOpen);
      
            /*
             * Configure the closed loop controller. We want to make sure we set the
             * feedback sensor as the primary encoder.
             */
            elevatorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(4200)
                .maxAcceleration(6000)
                .allowedClosedLoopError(0.5);
      
            // Configure basic settings of the intake motor
            intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
          }
        }
      
        public static final class AlgaeSubsystem {
          public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
          public static final SparkFlexConfig armConfig = new SparkFlexConfig();
      
          static {
            // Configure basic setting of the arm motor
            armConfig.smartCurrentLimit(40);
      
            /*
             * Configure the closed loop controller. We want to make sure we set the
             * feedback sensor as the primary encoder.
             */
            armConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed
                // loop slot, as it will default to slot 0.
                .p(0.1)
                .outputRange(-0.5, 0.5);
      
            // Configure basic settings of the intake motor
            intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
          }
        }
      }
}
