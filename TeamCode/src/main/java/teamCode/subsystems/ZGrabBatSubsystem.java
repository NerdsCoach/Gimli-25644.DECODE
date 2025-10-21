//package teamCode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class GrabBatSubsystem extends SubsystemBase
//{
//    private final Servo m_grabBatServo;
//
//    public GrabBatSubsystem(HardwareMap hMap, String name )
//    {
//        this.m_grabBatServo = hMap.get(Servo.class, name);
//    }
//
//    // Pivots position of the servo.
//    public void pivotGrabBatServo(double pos)
//    {
//        this.m_grabBatServo.setPosition(pos);
//    }
//}
