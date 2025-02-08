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

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


    public class ClimberSubsystem{
        XboxController driverController = new XboxController(0);
        private SparkMax climberMotor;
        private DigitalInput boreInput = new DigitalInput(1);
        private DutyCycleEncoder boreEncoder = new DutyCycleEncoder(boreInput);
        private double boreRawValue, boreConvertedValue, boreConvertedOffsetValue;
        private double speed = 0.025;
        private double targetLocation = 0/100;
        
        

        public  ClimberSubsystem() {
          climberMotor = new SparkMax(19, SparkLowLevel.MotorType.kBrushless);
          SparkMaxConfig config = new SparkMaxConfig();
          config
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.005)
          .d(0.001)
          .outputRange(-0.5, 0.5)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(4000)
          .allowedClosedLoopError(0.1);
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
            if (boreRawValue < targetLocation - .5) {
                climberMotor.set(speed);
            }   
            else if(boreRawValue > targetLocation + .5) {
                climberMotor.set(-speed);
            }
            else{
                climberMotor.set(0);
            }

            boreRawValue = boreEncoder.get();
            boreConvertedValue = boreRawValue * (360);
            boreConvertedOffsetValue = (boreConvertedValue - 106.084371);
            writeStatus();
        }
        private void writeStatus() {
        SmartDashboard.putNumber("boreConvertedOffsetValue",boreConvertedOffsetValue);
        SmartDashboard.putNumber("boreConvertedValue",boreConvertedValue);
        SmartDashboard.putNumber("boreRawValue",boreRawValue);
        }


        public void periodic(){
            
        }

        
    }   

