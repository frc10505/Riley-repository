package frc.team10505.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.team10505.robot.subsystems.AlgaeSubsystem;

public class RobotContainer {
    // Joysticks
    public final CommandJoystick joystick = new CommandJoystick(0);
    // private SendableChooser<Command> autonChooser;

    public RobotContainer() {
        simAlgaePivotControls();
        
        // autonChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Chooser",autonChooser);
        // configDefaultCommands();
    }

    // Subystems
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();

    // Pivot Controls
    private void simAlgaePivotControls() {
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsys.setAngle(0));
            joystick.button(2).onTrue(algaeSubsys.setAngle(-30));
            joystick.button(3).onTrue(algaeSubsys.setAngle(-90));
            joystick.button(4).onTrue(algaeSubsys.setAngle(90));

        }
    }

    // Flywheel Weird Intake Thing
}
