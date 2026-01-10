package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class SorterServoSubsystem extends SubsystemBase
{
    private final CRServo m_sorterServo;

    public SorterServoSubsystem(CRServo clock)
    {
        this.m_sorterServo = clock;
        this.m_sorterServo.setRunMode(Motor.RunMode.VelocityControl);
    }

    // Spins the intake wheel forward, or in reverse.
    public void spinSorter(double speed)
    {
        this.m_sorterServo.set(speed);
    }

}

