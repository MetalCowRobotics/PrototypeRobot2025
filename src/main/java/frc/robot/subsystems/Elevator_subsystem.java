package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.sim.SparkFlexSim;
// import com.revrobotics.sim.SparkLimitSwitchSim;
// import com.revrobotics.sim.SparkMaxSim;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.ElevatorSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.config.*;

public class Elevator_subsystem{}
//   /** Subsystem-wide setpoints */
//   public enum Setpoint {
//     kFeederStation,
//     kLevel1,
//     kLevel2,
//     kLevel3,
//     kLevel4;
// }
// private double armCurrentTarget;
// public static final class ElevatorSetpoints {
//     public static final int kFeederStation = 0;
//     public static final int kLevel1 = 0;
//     public static final int kLevel2 = 0;
//     public static final int kLevel3 = 100;
//     public static final int kLevel4 = 150;
//   }

//   // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
//   // initialize the closed loop controller and encoder.
//   private SparkMax armMotor =
//       new SparkMax(Elevator_subsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
//   private SparkClosedLoopController armController = armMotor.getClosedLoopController();
//   private RelativeEncoder armEncoder = armMotor.getEncoder();

//   // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
//   // need to initialize the closed loop controller and encoder.
//   private SparkFlex elevatorMotor =
//       new SparkFlex(Elevator_subsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
//   private SparkClosedLoopController elevatorClosedLoopController =
//       elevatorMotor.getClosedLoopController();
//   private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

//   // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
//   // controller like above.
//   private SparkMax intakeMotor =
//       new SparkMax(Elevator_subsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);

//   // Member variables for subsystem state management
//   private boolean wasResetByButton = false;
//   private boolean wasResetByLimit = false;
//   private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;

//   // Simulation setup and variables
//   private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
//   private SparkFlexSim elevatorMotorSim;
//   private SparkLimitSwitchSim elevatorLimitSwitchSim;

//   private DCMotor armMotorModel = DCMotor.getNEO(1);
  

  
//   public Elevator_subsystem() {
//     /*
//      * Apply the appropriate configurations to the SPARKs.
//      *
//      * kResetSafeParameters is used to get the SPARK to a known state. This
//      * is useful in case the SPARK is replaced.
//      *
//      * kPersistParameters is used to ensure the configuration is not lost when
//      * the SPARK loses power. This is useful for power cycles that may occur
//      * mid-operation.
//      */

//     elevatorMotor.configure(
//         config.CoralSubsystem.elevatorConfig,
//         ResetMode.kResetSafeParameters,
//         PersistMode.kPersistParameters);
        
//     // Zero arm and elevator encoders on initialization
//     elevatorEncoder.setPosition(0);

//     // Initialize simulation values
//     elevatorMotorSim = new SparkFlexSim(elevatorMotor, elevatorMotorModel);
//     elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
    

//   /**
//    * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
//    * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
//    * setpoints.
//    */
//   private void moveToSetpoint() {
//     armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
//     elevatorClosedLoopController.setReference(
//         elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
//   }

//   /** Zero the elevator encoder when the limit switch is pressed. */
//   private void zeroElevatorOnLimitSwitch() {
//     if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
//       // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
//       // prevent constant zeroing while pressed
//       elevatorEncoder.setPosition(0);
//       wasResetByLimit = true;
//     } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
//       wasResetByLimit = false;
//     }
//   }

//   /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
//   private void zeroOnUserButton() {
//     if (!wasResetByButton && RobotController.getUserButton()) {
//       // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
//       // constant zeroing while pressed
//       wasResetByButton = true;
//       armEncoder.setPosition(0);
//       elevatorEncoder.setPosition(0);
//     } else if (!RobotController.getUserButton()) {
//       wasResetByButton = false;
//     }
//   }

//   /** Set the intake motor power in the range of [-1, 1]. */
//   private void setIntakePower(double power) {
//     intakeMotor.set(power);
//   }

//   /**
//    * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
//    * positions for the given setpoint.
//    */
//   public Command setSetpointCommand(Setpoint setpoint) {
//     return this.runOnce(
//         () -> {
//           switch (setpoint) {
//             case kFeederStation:
//               armCurrentTarget = ArmSetpoints.kFeederStation;
//               elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
//               break;
//             case kLevel1:
//               armCurrentTarget = ArmSetpoints.kLevel1;
//               elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
//               break;
//             case kLevel2:
//               armCurrentTarget = ArmSetpoints.kLevel2;
//               elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
//               break;
//             case kLevel3:
//               armCurrentTarget = ArmSetpoints.kLevel3;
//               elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
//               break;
//             case kLevel4:
//               armCurrentTarget = ArmSetpoints.kLevel4;
//               elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
//               break;
//           }
//         });
//   }

//   /**
//    * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
//    * the motor will stop.
//    */
  
//   }

//   /**
//    * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
//    * released, the motor will stop.
//    */
//   public Command reverseIntakeCommand() {
//     return this.startEnd(
//         () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
//   }

//   @Override
//   public void periodic() {
//     moveToSetpoint();
//     zeroElevatorOnLimitSwitch();
//     zeroOnUserButton();
//   }
  

  

 
    

//     // Update sim limit switch

   
  

//     // SimBattery is updated in Robot.java
  
