package frc.team10505.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsytem extends SubsystemBase {
    /*Constants */
    final int kAlgaeIntakeMotorID = 7;

    /*Motor Controllers*/
    final SparkMax intakemotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
private SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig();
    /*Encoder*/

public AlgaeSubsytem (){
}
}
