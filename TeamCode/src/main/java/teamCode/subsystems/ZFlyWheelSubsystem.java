//package teamCode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class FlyWheelSubsystem extends SubsystemBase
//{
//    public DcMotor m_leftMotor;
//    public DcMotor m_rightMotor;
//
//    public void init(HardwareMap ahwMap)
//    {
//        m_leftMotor = ahwMap.get(DcMotor.class, "left_motor");
//        m_rightMotor = ahwMap.get(DcMotor.class, "right_motor");
//
////        // Set motor directions if needed
////        m_leftMotor.setDirection(DcMotor.Direction.REVERSE);
////        m_rightMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        // Set zero power behavior
//        m_leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m_rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//        public FlyWheelSubsystem(DcMotor m_leftMotor, DcMotor m_rightMotor)
//        {
//            this.m_leftMotor = m_leftMotor;
//            this.m_rightMotor = m_rightMotor;
//            this.m_leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            this.m_rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//    public void spin()
//    {
//        m_leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        m_rightMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        this.m_leftMotor.setPower(.8);
//        this.m_rightMotor.setPower(.8);
//    }
//}
//
////    public void flyWheel(int lift)
////    {
////        m_flyWheelMotor1.setTargetPosition(lift);
////        this.m_flyWheelMotor1.setPower(.7);//.7
////        this.m_flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////    }
