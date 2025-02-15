package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeSubsystem {
    private static IntakeSubsystem instance = new IntakeSubsystem();
    private SparkMax intakeMotor;
    private boolean intakeEnabled;
    private double speed = 0;
    private double pickUpSpeed = -0.6;
    private double holdSpeed = -0.05;
    private DigitalInput intakeSensor;
    private boolean alreadyStopped;

    private IntakeSubsystem() {
        intakeMotor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        intakeSensor = new DigitalInput(1);
        config.inverted(true);
        config.smartCurrentLimit(15);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        alreadyStopped = false;
        SmartDashboard.putBoolean("Intake Enabled", intakeEnabled);
    }

    public static IntakeSubsystem getInstance(){
        return instance;
    }

    public void periodic(){
        if(algaeAcquired()){
            intakeMotor.set(holdSpeed);
        }

        if(!algaeAcquired()){
            intakeMotor.set(speed);
        }

        SmartDashboard.putBoolean("Intake Enabled", intakeEnabled);
        SmartDashboard.putNumber("IntakeSpeed", speed);
        SmartDashboard.putBoolean("AlreadyStopped", alreadyStopped);
    }

    public boolean algaeAcquired(){
        return !intakeSensor.get();
    }

   
    public void startIntake(){
        speed = pickUpSpeed;
        System.out.println("Intake Enabled");
    }

    public void stopIntake(){
        if(algaeAcquired()){
            releaseAlgae();
        }

        speed = 0;

        System.out.println("Intake Disabled");
    }

    public void releaseAlgae(){
        intakeMotor.set(1);

        if(!algaeAcquired()){
            Commands.waitSeconds(3);
            speed = 0;
        }else{
            releaseAlgae();
        }
    }

    public void startIntakeReverse(){
        speed = 1;
        intakeEnabled = true;
    }

    public double getSpeed(){
        return speed;
    }
}
