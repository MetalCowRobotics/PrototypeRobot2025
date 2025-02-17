package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;

/*This command toggles the shooter on/off */
public class RunIntake extends Command{
    @Override
    public void execute(){
        IntakeSubsystem.getInstance().startIntake();
        SmartDashboard.putString("auto", "running");
    }
}
