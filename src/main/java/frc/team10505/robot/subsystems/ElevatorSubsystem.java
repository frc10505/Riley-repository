package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // Variables
    private TalonFX follower;
    private TalonFX lead;
    private PIDController elevatorController;
    private ElevatorFeedforward elevatorFeedForward;

    private double setpoint = 0;

    public ElevatorSubsystem() {
        if (Utils.isSimulation()) {
            follower = new TalonFX(1);
            lead = new TalonFX(2);
            elevatorController = new PIDController(0, 0, 0);
            elevatorFeedForward = new ElevatorFeedforward(0, 0, 0);
        } else {
            follower = new TalonFX(0, getName());
            lead = new TalonFX(0, getName());
            elevatorController = new PIDController(0, 0, 0);
            elevatorFeedForward = new ElevatorFeedforward(0, 0, 0);
        }
    }

    public Command setpoint() {
        return runEnd(null, null);
    }

    @Override
    public void periodic() {

    }
}
