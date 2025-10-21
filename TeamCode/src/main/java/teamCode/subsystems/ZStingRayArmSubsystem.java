//package teamCode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class StingRayArmSubsystem extends SubsystemBase
//{
//    private final Servo m_ascentArmServo;
//
//    public StingRayArmSubsystem(HardwareMap hMap, String name )
//    {
//        this.m_ascentArmServo = hMap.get(Servo.class, name);
//    }
//
//    // Pivots position of the intake.
//    public void ascentArm(double pos)
//    {
//        this.m_ascentArmServo.setPosition(pos);
//    }
//}
