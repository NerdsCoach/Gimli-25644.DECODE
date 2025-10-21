package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LegolasLauncherSubsystem extends SubsystemBase
{
    private final DcMotor m_launcherMotor;

    public LegolasLauncherSubsystem (DcMotor launcherMotor)
    {
       this.m_launcherMotor = launcherMotor;
       this.m_launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       this.m_launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       this.m_launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void launch (int launch)
    {
        this.m_launcherMotor.setTargetPosition(launch);
        this.m_launcherMotor.setPower(0.8);
        this.m_launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stop()
    {
        this.m_launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
