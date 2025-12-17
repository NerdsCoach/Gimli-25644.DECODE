package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherServoSubsystem extends SubsystemBase
{
    private final Servo m_launcherServo;

    public LauncherServoSubsystem(HardwareMap hMap, String name )
    {
        this.m_launcherServo = hMap.get(Servo.class, name);
    }

    // Pivots position of the intake.
    public void ascentArm(double pos)
    {
        this.m_launcherServo.setPosition(pos);
    }
}
