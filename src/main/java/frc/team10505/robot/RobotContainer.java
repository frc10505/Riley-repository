package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.team10505.robot.subsystems.AlgaeSubsystem;

public class RobotContainer {
 //Joysticks
public final CommandJoystick joystick = new CommandJoystick(1);

//Subystems
private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
//Controls
private void simAlgaePivotControls(){
    if(Utils.isSimulation()){
        //joystick.button(1).onTrue(algaeSubsys.setAngle)
    } 
}

}
