package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LauncherSubsystem extends SubsystemBase
{
    private final DcMotorEx m_launcherMotorRed;
    private double launcherTargetVelocity = 1000;

    public LauncherSubsystem(DcMotorEx launcherMotor)
    {
        this.m_launcherMotorRed = launcherMotor;
        this.m_launcherMotorRed.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.m_launcherMotorRed.setDirection(DcMotorEx.Direction.FORWARD);
        this.m_launcherMotorRed.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorVelocity(double ticksPerSec)
    {
        this.m_launcherMotorRed.setVelocity(ticksPerSec);
    }

    public void stop()
    {
        this.m_launcherMotorRed.setVelocity(0);
        this.m_launcherMotorRed.setPower(0);
    }

    public void fudgeFactor(int fudge)
    {
        this.m_launcherMotorRed.setDirection(DcMotorEx.Direction.FORWARD);

        if (m_launcherMotorRed.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            m_launcherMotorRed.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        this.launcherTargetVelocity += fudge;
        m_launcherMotorRed.setVelocity(this.launcherTargetVelocity);
//        m_launcherMotorRed.setVelocity(m_launcherMotorRed.getVelocity() + fudge);
    }

    public double getVel ()
    {
        return this.m_launcherMotorRed.getVelocity();
    }


    // Check if the motor is actually spinning near the speed we want
    public boolean atTarget(double target)
    {
        return Math.abs(m_launcherMotorRed.getVelocity() - target) < 50; // Tolerance of 50 ticks
    }
}
