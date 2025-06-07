package frc.team10505.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.AlgaeSubsystem;

public class RobotContainer {
    // Joysticks
    public final CommandJoystick joystick = new CommandJoystick(0);
    public final CommandJoystick joystick2 = new CommandJoystick(1);
    public final CommandXboxController driveController = new CommandXboxController(1);
    // private SendableChooser<Command> autonChooser;

    public RobotContainer() {
        algaePivotControls();
        simAlgaeIntakeControls();
        // autonChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Chooser",autonChooser);
        // configDefaultCommands();
    }

    // Subystems
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();

    // Pivot Controls
    private void algaePivotControls() {
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsys.setAngle(0));
            joystick.button(2).onTrue(algaeSubsys.setAngle(-30));
            joystick.button(3).onTrue(algaeSubsys.setAngle(-90));
        } //else {
            // driveController.x().onTrue(algaeSubsys.setAngle(0));
            // driveController.a().onTrue(algaeSubsys.setAngle(-30));
            // driveController.b().onTrue(algaeSubsys.setAngle(-90));
        //}
    }

    // Flywheel thing
    private void simAlgaeIntakeControls() {
        if (Utils.isSimulation()) {
            joystick2.button(1).whileTrue(algaeSubsys.runIntake(.5));
            joystick2.button(2).whileTrue(algaeSubsys.runIntake(-0.5));

        } //else {
        //     driveController.leftBumper().whileTrue(algaeSubsys.runIntake(.5));
        //     driveController.rightBumper().whileTrue(algaeSubsys.runIntake(-0.5));
        // }
    }
}
