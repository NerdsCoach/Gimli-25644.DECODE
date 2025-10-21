package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class SecondBreakfastIntakeSubsystem extends SubsystemBase
{
    private final CRServo m_intakeServo;

    public SecondBreakfastIntakeSubsystem(CRServo intake)
    {
        this.m_intakeServo = intake;
        this.m_intakeServo.setRunMode(Motor.RunMode.VelocityControl);
    }

    // Spins the intake wheel forward, or in reverse.
    public void spinIntake(double speed)
    {
        this.m_intakeServo.set(speed);
    }

    public void spinIntake(int spin)
    {
        this.m_intakeServo.setTargetPosition(spin);
    }
    public void stop()
    {
        this.m_intakeServo.stop();
    }
}

