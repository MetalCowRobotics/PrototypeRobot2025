package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class NoteTransitSubsystem {
    private static NoteTransitSubsystem instance = new NoteTransitSubsystem();
    private IntakeJointSubsystem m_IntakeJointSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    private double intakeTarget;
    private enum positions {rest, pickup, shortshot, longshot, amp};


    private NoteTransitSubsystem(){
        m_IntakeJointSubsystem = IntakeJointSubsystem.getInstance();
        m_IntakeSubsystem = IntakeSubsystem.getInstance();
    }

    public static NoteTransitSubsystem getInstance(){
        return instance;
    } 

    public boolean atTarget(){
        SmartDashboard.putBoolean("atTarget", m_IntakeJointSubsystem.atTarget());
        return m_IntakeJointSubsystem.atTarget();
    }

    //Sets joints to pickup location, and sets the intake speed to the pickup speed for when it is enabled,
    public void setPickupPosition(){
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakeDeployed);
    }

   
    //Sets joints to rest location, and sets the intake speed to the off 
    public void setRestPosition(){
        m_IntakeJointSubsystem.setTarget(Constants.JointConstants.intakeStart);
        m_IntakeSubsystem.stopIntake();
    }

    //Turns on the intake to the speed that the state requires, except if you are in a state where you are shooting, then if you try, it does not enable the intake unless the shooter is at speed
    public void enableIntake(){
            m_IntakeSubsystem.startIntake();
    }

    //Spits out a piece regardless of position, ideal to be done in amp position
    public void quickOuttake(){
        m_IntakeSubsystem.startIntakeReverse();
    }

    //Turns off the intake
    public void disableIntake(){
        m_IntakeSubsystem.stopIntake();
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Target", intakeTarget);
        SmartDashboard.putBoolean("atTarget", m_IntakeJointSubsystem.atTarget());
        //Automatically lifts the intake once tehre is a note inside
        // if(m_IntakeSubsystem.getReadyToLift() && !alreadyLiftedIntake){
        //     setSpeakerPosition();
        //     alreadyLiftedIntake = true;
        // }
        m_IntakeSubsystem.periodic();
        m_IntakeJointSubsystem.periodic();
    }
}