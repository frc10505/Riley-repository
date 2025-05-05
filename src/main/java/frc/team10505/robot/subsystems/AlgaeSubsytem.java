package frc.team10505.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsytem extends SubsystemBase {
    /* Constants */
    final int algaeIntakeMotorID = 7;
    private double startingAngle = 0;
    private final int pivotMotorId = 20;
    private final int intakeMotorId = 21;
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

    /* Motor Controllers */
    final SparkMax intakemotor = new SparkMax(algaeIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig();
    /* Motor Ids */

    private SparkMax pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);

    private SparkMax intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);

    public AlgaeSubsytem() {
    }

    /* Sim Variables */
    private final Mechanism2d pivotMech = new Mechanism2d(2, 2);
    private MechanismRoot2d pivotRoot = pivotMech.getRoot("PivotRoot", 1, 1);
    private MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("PivotViz", .7, 0));
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2),
            0.305, Units.degreesToRadians(-110), Units.degreesToRadians(-110), true, startingAngle);

    // /*Pivot Motor Config */
    // pivotMotorConfig.idleMode(IdleMode.kBrake);
    // pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit,
    // kPivotMotorCurrentLimit);
    // pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale);
    // // Angle encoder scale
    // pivotMotorConfig.absoluteEncoder.zeroOffset(pivotEncoderOffset); // Angle
    // encoder offset
    // pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
}
