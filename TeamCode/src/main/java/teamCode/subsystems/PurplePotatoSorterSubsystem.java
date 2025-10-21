package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PurplePotatoSorterSubsystem extends SubsystemBase
{
    private final Servo m_sorterServo;

    public PurplePotatoSorterSubsystem(HardwareMap hMap, String name)
    {
        this.m_sorterServo = hMap.get(Servo.class, name);
    }

    // Pivots position of the intake.
    public void spinServo(double pos)
    {
        this.m_sorterServo.setPosition(pos);
    }
}
