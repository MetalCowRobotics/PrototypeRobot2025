package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command toggles the shooter on/off */
public class SetLevel2 extends Command{
    @Override
    public void execute(){
        ElevatorSubsystem.getInstance().L2_Distance();
        SmartDashboard.putString("auto", "running");
    }
}
