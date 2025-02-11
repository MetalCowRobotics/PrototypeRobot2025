package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


    public class ClimberSubsystem{
        XboxController driverController = new XboxController(0);
        private SparkMax climberMotor;
        private DigitalInput boreInput = new DigitalInput(2);
        private DutyCycleEncoder boreEncoder = new DutyCycleEncoder(boreInput);
        private double boreRawValue, boreConvertedValue, boreConvertedOffsetValue;
        private double speed = 0.1;
        private double targetLocation = 0;

        
        

        public  ClimberSubsystem() {
          climberMotor = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
          SparkMaxConfig config = new SparkMaxConfig();
          config
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.005)
          .d(0.001)
          .outputRange(-0.1, 0.1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(4000)
          .allowedClosedLoopError(0.1);
          climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        
        public void runClimblerSpeed(){
            climberMotor.set(speed);    
        }

        public double getCurrentAngle(){
        return boreConvertedOffsetValue;
        }

        public void setTargetLocation(double targetlocation){
            this.targetLocation = targetLocation;
            
        }

        public void HarpoonOut(){
            targetLocation = 0.75;
        }

        public void HarpoonIn(){
            targetLocation = 0.25;
        }

        public void runClimber(){
            boreRawValue = boreEncoder.get();
            if (boreRawValue < targetLocation - .1) {
                climberMotor.set(-speed);
            }   
            else if(boreRawValue > targetLocation + .1) {
                climberMotor.set(speed);
            }
            else{
                climberMotor.set(0);
            }

            boreRawValue = boreEncoder.get();
            boreConvertedValue = boreRawValue * (360);
            writeStatus();
            pushValue();
        }
        private void writeStatus() {
        SmartDashboard.putNumber("boreConvertedOffsetValue",boreConvertedOffsetValue);
        SmartDashboard.putNumber("boreConvertedValue",boreConvertedValue);
        SmartDashboard.putNumber("boreRawValue",boreRawValue);
        }


        public void periodic(){
            pushValue();
            runClimber();
        }

        private void pushValue(){
            // SmartDashboard.putNumber("Climb Motor Speed", climberMotor.speed);
            SmartDashboard.putNumber("Encoder Reading", boreEncoder.get());
            SmartDashboard.putNumber("Target Location", targetLocation);
            
        } 

        
    }   

