package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitchSubsystem extends SubsystemBase
{
    private final CRServo m_transferServo;
    private final RevTouchSensor m_limitSwitch;

    // DECLARE THE VARIABLE HERE to fix the error
    private int hitCount = 0;

    public LimitSwitchSubsystem(RevTouchSensor limitSwitch, CRServo transferServo)
    {
        m_limitSwitch = limitSwitch;
        m_transferServo = transferServo;
    }

    public void setPower(double speed)
    {
        m_transferServo.set(speed);
    }

    @Override
    public void periodic()
    {
        // This will force the telemetry to show the real-time state
        // (Assuming you pass telemetry into the subsystem or use a global one)
        // If the switch is working, this should flip to 'true' when you click it.
    }

    public boolean isPressed()
    {
        return m_limitSwitch.isPressed();

    }

    // These will now work because hitCount is defined above
    public int getHitCount()
    {
        return hitCount;
    }
    public void incrementHits()
    {
        hitCount++;
    }
    public void resetHits()
    {
        hitCount = 0;
    }

    public void stop()
    {
        m_transferServo.stop();
    }

}
