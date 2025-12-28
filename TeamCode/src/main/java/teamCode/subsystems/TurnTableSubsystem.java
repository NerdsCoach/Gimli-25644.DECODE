package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurnTableSubsystem extends SubsystemBase
{
        private final DcMotor m_turnTableMotor;
        private static final double TURRET_SPEED = 0.5;

    public TurnTableSubsystem(DcMotor turnTableMotor)
    {
        m_turnTableMotor = turnTableMotor;
        m_turnTableMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

        public void turn(double speed)
    {
        m_turnTableMotor.setPower(speed * TURRET_SPEED);
    }

        public void stop()
    {
        m_turnTableMotor.setPower(0);
    }
//    private final DcMotor m_turnTableMotor;
//
//    public TurnTableSubsystem(DcMotor turnTableMotor)
//    {
//        this.m_turnTableMotor = turnTableMotor;
////        motor = hardwareMap.get(DcMotor.class, "aim_motor"); // Name from config
//        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoders for better control
//        this.m_turnTableMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.m_turnTableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void setMotorPower(double power)
//    {
//        m_turnTableMotor.setPower(power);
//    }
//
//    public void stopMotor() {
//        m_turnTableMotor.setPower(0);
//    }
////    private final DcMotor m_turnTableMotor;
////    public static final double PAN_CENTER = 0.5;
////    public static final double PAN_MAX = 1.0;
////    public static final double PAN_MIN = 0.0;
////
////    public TurnTableSubsystem(DcMotor turnTableMotor)
////    {
////        this.m_turnTableMotor = turnTableMotor;
////        this.m_turnTableMotor.setDirection(DcMotorSimple.Direction.REVERSE);
////        this.m_turnTableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        this.m_turnTableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////    }
////
//     public void turnTable()
//    {
////        this.m_turnTableMotor.setTargetPosition(this.m_turnTableMotor.getCurrentPosition() + turn);
//
////        this.m_turnTableMotor.setTargetPosition(turn);
//        this.m_turnTableMotor.setPower(.1);
//        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    public boolean atTarget(double target)
//    {
//        return this.m_turnTableMotor.getCurrentPosition() <= target+5 && this.m_turnTableMotor.getCurrentPosition() >= target-5;
//    }
//
////    public boolean atTarget(double target)
////    {
////        return this.m_turnTableMotor.getCurrentPosition() <= target+5 && this.m_turnTableMotor.getCurrentPosition() >= target-5;
////    }
//
//    public double setPanPosition()
//    {
//        return  m_turnTableMotor.getCurrentPosition();
//    }
//
//    public double getPanPosition()
//    {
//        return m_turnTableMotor.getCurrentPosition();
//    }
//
//    public void stop()
//    {
//        this.m_turnTableMotor.setPower(0);
//        this.m_turnTableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.m_turnTableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
}
