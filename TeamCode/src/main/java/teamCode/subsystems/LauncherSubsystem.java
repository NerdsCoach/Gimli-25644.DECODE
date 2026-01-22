package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LauncherSubsystem extends SubsystemBase
{
    private final DcMotorEx m_launcherMotor;
//    private final DcMotor m_launcherMotorBlue;

    public LauncherSubsystem(DcMotorEx launcherMotor)
    {
       this.m_launcherMotor = launcherMotor;
       this.m_launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//               (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       this.m_launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
       this.m_launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void launch ()
    {
//        this.m_launcherMotor.setPower(0.3); //1 at far 0.75
        this.m_launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.m_launcherMotor.setVelocity(2450);//5800RPM Max Output  (5800RPM/60)*28 // 2700
        // 6 ft = 2300

        // 6 inch from goal = 1700

    }

    public void stop()
    {
        this.m_launcherMotor.setPower(0);
        this.m_launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean atTarget()
    {
        return this.m_launcherMotor.getPowerFloat();
    }
}
