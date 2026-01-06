package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeServoSubsystem extends SubsystemBase
{
//   .
    private final CRServo m_intakeServo;
    // private final ColorSensor m_colorSensor;

    //public ColorSensorSubsystem(ColorSensor color)
    // {
        //this.m_colorSensor;
        //this.m_colorSensor.setMode(ColorSensing);
    // }

    public IntakeServoSubsystem(CRServo wheel)
    {
        this.m_intakeServo = wheel;
        this.m_intakeServo.setRunMode(Motor.RunMode.VelocityControl);
    }

    // Spins the intake wheel forward, or in reverse.
    public void spinIntake(double speed)
    {
        this.m_intakeServo.set(speed);
    }
//    public void spinIntake(int spin)
//    {
//        this.m_intakeServo.setTargetPosition(spin);
//    }
}

