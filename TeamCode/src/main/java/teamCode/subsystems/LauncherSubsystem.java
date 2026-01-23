package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LauncherSubsystem extends SubsystemBase
{
    private final DcMotorEx m_launcherMotorRed;


    public LauncherSubsystem(DcMotorEx launcherMotor)
    {
        this.m_launcherMotorRed = launcherMotor;
        // MUST use RUN_USING_ENCODER for .setVelocity() to work!
        this.m_launcherMotorRed.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.m_launcherMotorRed.setDirection(DcMotorEx.Direction.FORWARD);
        this.m_launcherMotorRed.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorVelocity(double ticksPerSec)
    {
        this.m_launcherMotorRed.setVelocity(ticksPerSec);
    }

    public void stop() {
        this.m_launcherMotorRed.setVelocity(0);
        this.m_launcherMotorRed.setPower(0);
    }

    // Check if the motor is actually spinning near the speed we want
    public boolean atTarget(double target)
    {
        return Math.abs(m_launcherMotorRed.getVelocity() - target) < 50; // Tolerance of 50 ticks
    }
//
//    public double getTargetVelocity(double distance)
//    {
//        // Find points for linear interpolation
//        Double lowKey = velocityLUT.floorKey(distance);
//        Double highKey = velocityLUT.ceilingKey(distance);
//        if (lowKey == null) return velocityLUT.get(highKey);
//        if (highKey == null) return velocityLUT.get(lowKey);
//        if (lowKey.equals(highKey)) return velocityLUT.get(lowKey);
//
//        // Linear Interpolation: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
//        return velocityLUT.get(lowKey) + (distance - lowKey) *
//                (velocityLUT.get(highKey) - velocityLUT.get(lowKey)) / (highKey - lowKey);
//    }
//
//
//    public void launch ()
//    {
////        this.m_launcherMotorRed.setPower(0.3); //1 at far 0.75
//        this.m_launcherMotorRed.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        this.m_launcherMotorRed.setVelocity(2450);//5800RPM Max Output  (5800RPM/60)*28 // 2700
//    }
//
//    public void stop()
//    {
//        this.m_launcherMotorRed.setPower(0);
//        this.m_launcherMotorRed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//
//    public boolean atTarget()
//    {
//        return this.m_launcherMotorRed.getPowerFloat();
//    }
}
