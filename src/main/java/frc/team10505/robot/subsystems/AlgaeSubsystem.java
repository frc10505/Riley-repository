package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    /* Constants */
    final int algaeIntakeMotorID = 7;
    private double startingAngle = 0;
    private final int pivotMotorId = 20;
    private final int intakeMotorId = 21;
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private PIDController pivotController;
    private ArmFeedforward pivotFeedForward;
    public final static int ALGAE_INTAKE_MOTOR_ID = 8;
    
    /* Motor Controllers */
    final SparkMax intakemotor = new SparkMax(algaeIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig();
    /* Motor Ids */

    private SparkMax pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);

    /* Sim Variables */
    private final Mechanism2d pivotMech = new Mechanism2d(2, 2);
    private MechanismRoot2d pivotRoot = pivotMech.getRoot("PivotRoot", 0.75, 0.75);
    private MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("PivotViz", .7, 0));
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2),
            0.305, Units.degreesToRadians(-110), Units.degreesToRadians(-110), true, startingAngle);

    private final Mechanism2d intakeMech = new Mechanism2d(2, 2);
    private MechanismRoot2d intakeRoot = pivotMech.getRoot("IntakeRoot", 1, 1);
    private MechanismLigament2d intakeViz = pivotRoot.append(new MechanismLigament2d("IntakeViz", .7, 0));
    private FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
            0.000000000001, 4), DCMotor.getNEO(1), 6);

    private final int kPivotMotorCurrentLimit = 1;
    private final double pivotEncoderScale = 1;
    private double pivotSetpoint = -90;
    private final SparkMax intakeMotor = new SparkMax(ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);
    private double intakeSpeed = 0;
    public boolean coasting = false;

    // Constructor
    public AlgaeSubsystem() {
        SmartDashboard.putData("Pivot Sim", pivotMech);
        if (Utils.isSimulation()) {
            pivotController = new PIDController(0, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);
            pivotMotorConfig = new SparkMaxConfig();
            intakeMotorConfig = new SparkMaxConfig(); 
        }
         /* Pivot Configurator */
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit, kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale);
       
        if(Utils.isSimulation() || Utils.isReplay()){
            pivotController = new PIDController(1.6, 0, 0.01);
            pivotFeedForward = new ArmFeedforward(0, 0.1719, 0.4, 0.1);
        }else{
            pivotController = new PIDController(0.1, 0, 0);
            pivotFeedForward = new ArmFeedforward(0.01, 0.1, 0.4, 0.1);
        }
    }

    public Command setAngle(double Angle) {
        return run(() -> {
            pivotSetpoint = Angle;
        });
    }
    public Command setVoltage(double Voltage) {
        return run(() -> {
            pivotMotor.setVoltage(Voltage);
        });
    }

    // Periodic Stuff
    @Override
    public void periodic() {
        // dashboard stuff
        SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
        // SmartDashboard.putNumber("Pivot Encoder", getPivotEncoder());
        // SmartDashboard.putNumber("Pivot Calculated Effort", getEffort());
        SmartDashboard.putNumber("Algae Intake Speed", intakeSpeed);

        // Sim Updating
        if (Utils.isSimulation() || Utils.isReplay()) {
          // pivotSim.setInput(getEffort());
            pivotSim.update(0.01);
            pivotViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));

            intakeSim.setInput(intakeSpeed);
            intakeSim.update(0.01);
            intakeViz.setAngle(Units.radiansToDegrees(intakeViz.getAngle() + (intakeSpeed * 0.05)));
            intakeRoot.setPosition((Math.cos(Units.degreesToRadians(pivotViz.getAngle())) * 0.56) + 0.75,
                    (Math.sin(Units.degreesToRadians(pivotViz.getAngle())) * 0.56) + 0.75);

            SmartDashboard.putNumber("Sim Algae Intake Viz Angle", intakeViz.getAngle());

        } else {
            if (!coasting) {
                //pivotMotor.setVoltage(getEffort());
            }
            SmartDashboard.putNumber("Algae Intake Motor Output", intakeMotor.getAppliedOutput());
            SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
        }
    }

    // private double getPivotEncoder() {
    //     if(Utils.isSimulation()){
    //         return pivotViz.getAngle();
    //     }else{
    //         return (-pivotEncoder.getPosition() + absoluteOffset);
    //     }
    // }

}