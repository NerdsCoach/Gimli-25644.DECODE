//package teamCode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class AimingServoSubsystem extends SubsystemBase
//{
////    private final Servo panServo; // Servo for horizontal aiming
//    private final Servo m_aimingServo;
//
//    // Define the range of motion for the servo
//    public static final double PAN_CENTER = 0.5;
//    public static final double PAN_MAX = 1.0;
//    public static final double PAN_MIN = 0.0;
//
//    public AimingServoSubsystem(HardwareMap hMap, String transferServo)
//    {
//        this.m_aimingServo = hMap.get(Servo.class, transferServo);
//    }
//
//    public void setPanPosition(double position)
//    {
//        m_aimingServo.setPosition(position);
//    }
//
//    public double getPanPosition()
//    {
//        return m_aimingServo.getPosition();
//    }
//}
