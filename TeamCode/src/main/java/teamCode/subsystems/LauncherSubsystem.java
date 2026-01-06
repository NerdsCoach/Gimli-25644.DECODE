package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LauncherSubsystem extends SubsystemBase
{
    private final DcMotor m_launcherMotor;
//    private final DcMotor m_launcherMotorBlue;

    public LauncherSubsystem(DcMotor launcherMotor)
    {
       this.m_launcherMotor = launcherMotor;
       this.m_launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       this.m_launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
       this.m_launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void launch ()
    {
        this.m_launcherMotor.setPower(0.45); //1 at far 0.75
        this.m_launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
