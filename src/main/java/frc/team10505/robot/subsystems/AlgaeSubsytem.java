package frc.team10505.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsytem extends SubsystemBase {
    /*Constants */
    final int kAlgaeIntakeMotorID = 7;

    /*Motors*/
    final SparkMax intakemotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
private SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig(kAlgaeIntakeMotorID, null)
    /*Encoder*/

public AlgaeSubsytem (){
}
}
