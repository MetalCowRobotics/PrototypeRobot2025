package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.config.Configs;
import frc.robot.Constants;


public class ElevatorSubsystem {
    private double defaultMotorSpeed = 0.25;
    //public double kFeederStation = 20;
    private double motorSpeed = defaultMotorSpeed; 
    private double targetLocation = 0;
    
    private SparkMax ElevatorMotor = new SparkMax(21, MotorType.kBrushless);
    private SparkClosedLoopController elevatorClosedLoopController = ElevatorMotor.getClosedLoopController();
    private SparkLimitSwitch BottomSwitch = ElevatorMotor.getReverseLimitSwitch();
    private SparkLimitSwitch TopSwitch = ElevatorMotor.getForwardLimitSwitch();

    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true);
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
            
      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      config
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen)
          .forwardLimitSwitchEnabled(true)
          .forwardLimitSwitchType(Type.kNormallyOpen);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      config
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.15)
          .d(0.01)
          .outputRange(-0.5, 0.5)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(4000)
          .allowedClosedLoopError(0.25);

        ElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        zeroEncoder();
    }
    public void periodic() {
        pushValue();

        
        // if (!BottomSwitch.get()) {
        //     zeroEncoder();    
        // } 
        
        // if (getPosition() <= targetLocation) {
        //     climbMotor.set(defaultMotorSpeed);
        // } else {
        //     climbMotor.set(0);
        // }
        // if (getPosition() >= targetLocation + 3) {
        //     climbMotor.set(-defaultMotorSpeed);
        
        // } else if (getPosition() <= targetLocation - 3) {
        //     climbMotor.set(defaultMotorSpeed);
        // } else {
        //     climbMotor.set(0);
        // }
        
    }
    private double getPosition() {
        return ElevatorMotor.getEncoder().getPosition();
    }   
    public void setTargetLocation(double targetLocation) {
        this.targetLocation = targetLocation;
        elevatorClosedLoopController.setReference(
        targetLocation, ControlType.kMAXMotionPositionControl);
    }
    private void pushValue(){
        SmartDashboard.putNumber("Climb Motor Speed", motorSpeed);
        SmartDashboard.putNumber("Encoder Reading", getPosition());
        SmartDashboard.putNumber("Target Location", targetLocation);
        SmartDashboard.putBoolean("Bottom Switch", BottomSwitch.isPressed());
        SmartDashboard.putBoolean("Top Switch", TopSwitch.isPressed());
        
    }   
    public void zeroEncoder(){
        ElevatorMotor.getEncoder().setPosition(0);
    }
   public void L1_Distance(){
    targetLocation = Constants.L1_Distance;
   }

   public void L2_Distance(){
    targetLocation = Constants.L2_Distance;
   }

   public void L3_Distance(){
    targetLocation = Constants.L3_Distance;
   }

   public void L4_Distance(){
    targetLocation = Constants.L4_Distance;
   }
    public void FeederStation(){
     targetLocation = Constants.L1_Distance;
    }
  
   

   //Equation = encoder1.GetPosition * (130.1247*(1/15)/42)
}





//     // Default motor speed
//     private double motorSpeed = 0.1; // Default speed, can be changed using setSpeed()

//     // Button ID for motor activation (use an appropriate button ID)
//     private static final int ACTIVATE_BUTTON = XboxController.Button.kA.value; // Button A
//     SparkMax climbMotor = new SparkMax(19, MotorType.kBrushless);
         
//     SparkMax climbmotor1 = new SparkMax(21, MotorType.kBrushless);
//     // Constructor
    




// public class Robot extends TimedRobot {
//     private CANSparkMax  = new CANSparkMax(19, SparkLowLevel.MotorType.kBrushless);
//         // Initialize the motor with the CAN ID and brushless motor type
//         //     SparkMaxConfig config = new SparkMaxConfig();
//         //     config
//         //     .inverted(true)
//         //     .idleMode(IdleMode.kCoast);
            
//         //     climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         //     SparkMaxConfig config1 = new SparkMaxConfig();
//         //     config1.idleMode(IdleMode.kCoast)
//         //     .inverted(false);
//         //     // .follow(climbMotor);
            
            
//         //     climbmotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            
//         //     // .idleMode(IdleMode.kBrake);
            
    

//         // // Restore default settings to ensure a clean configuration
//         // // climbMotor.restoreFactoryDefaults();

//         // // Initialize the Xbox controller (port 0 by default)
//         // controller = new XboxController(0);
//     }

//     // Method to control the motor
//     public void controlMotor() {
//         // Check if the activation button is pressed
//         // if (controller.getRawButton(ACTIVATE_BUTTON)) {
//             // Move the motor at the specified speed
//             //climbMotor.set(motorSpeed);
//             climbmotor1.set(motorSpeed);
//             climbMotor.set(motorSpeed);
//         }
//     // }

//     // Method to set the motor speed
//     public void setSpeed(double speed) {
//         // Ensure the speed is within the valid range [-1, 1]
//         motorSpeed = Math.max(-1, Math.min(1, speed));
//     }
// }



// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.XboxController;

// public class climb {

//     // Xbox Controller--
//     private final XboxController controller;


//     // Default motor speed
//     private double motorSpeed = 0.1; // Default speed, can be changed using setSpeed()

//     // Button ID for motor activation (use an appropriate button ID)
//     private static final int ACTIVATE_BUTTON = XboxController.Button.kA.value; // Button A
//     SparkMax climbMotor = new SparkMax(19, MotorType.kBrushless);
         
//     SparkMax climbmotor1 = new SparkMax(21, MotorType.kBrushless);
//     // Constructor
//     public climb() {
//         // Initialize the motor with the CAN ID and brushless motor type
//             SparkMaxConfig config = new SparkMaxConfig();
//             config
//             .inverted(true)
//             .idleMode(IdleMode.kCoast);
            
            
//             climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//             SparkMaxConfig config1 = new SparkMaxConfig();
//             config1.idleMode(IdleMode.kCoast)
//             .inverted(false);
//              SparkBaseConfig config1base = config1;
//              config1base.follow(climbMotor);
         
//             climbmotor1.configure(config1base, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            
//             // .idleMode(IdleMode.kBrake);
            
    

//         // Restore default settings to ensure a clean configuration
//         // climbMotor.restoreFactoryDefaults();

//         // Initialize the Xbox controller (port 0 by default)
//         controller = new XboxController(0);
//     }

//     // Method to control the motor
//     public void controlMotor() {
//         // Check if the activation button is pressed
//         // if (controller.getRawButton(ACTIVATE_BUTTON)) {
//             // Move the motor at the specified speed
//             //climbMotor.set(motorSpeed);
//             climbmotor1.set(motorSpeed);
//             climbMotor.set(motorSpeed);
//         }
//     // }

//     // Method to set the motor speed
//     public void setSpeed(double speed) {
//         // Ensure the speed is within the valid range [-1, 1]
//         motorSpeed = Math.max(-1, Math.min(1, speed));
//     }
// }
