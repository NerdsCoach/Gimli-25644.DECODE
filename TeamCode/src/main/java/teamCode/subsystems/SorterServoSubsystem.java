package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class SorterServoSubsystem extends SubsystemBase
{
    private final Servo m_sorterServo;

    public SorterServoSubsystem(HardwareMap hMap, String sorterServo)
    {
//        this.m_sorterServo = hMap.get(Servo.class, sorterServo);
        this.m_sorterServo = hMap.get(ServoImplEx.class, "sorterServo");
    }

    // Pivots position of the intake.
    public void sort(double pos)
    {
        this.m_sorterServo.setPosition(pos);
    }
    public boolean atTarget(double target)
    {
        return this.m_sorterServo.getPosition() >= target-5 && this.m_sorterServo.getPosition() <= target+5;
    }
}