package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import java.lang.reflect.Method;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class MotorSubsystem extends SubsystemBase {
//double wantedSpeed =  0.21;

    private static CANSparkMax intakeMotor = new CANSparkMax(1, MotorType.kBrushed);


    public MotorSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        
    }
        

    public void run(double speedInput) {
      intakeMotor.set(speedInput);
        }

    public void off() {
      intakeMotor.set(0);
        }
} 




























// "P.I.D" more like "Party In Diddy's domicile".


