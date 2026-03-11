package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevTouchSensor;

public class IntakeServoSubsystem extends SubsystemBase
{

    private final CRServo m_intakeServo;
    private final RevTouchSensor m_intakelimitSwitch;
    private int hitCount = 0;

    public IntakeServoSubsystem(RevTouchSensor intakeLimitSwitch, CRServo wheel)
    {
        this.m_intakeServo = wheel;
        m_intakelimitSwitch = intakeLimitSwitch;
        this.m_intakeServo.setRunMode(Motor.RunMode.VelocityControl);
    }

    // Spins the intake wheel forward, or in reverse.
    public void spinServo(double speed)
    {
        this.m_intakeServo.set(speed);
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
        return m_intakelimitSwitch.isPressed();

    }

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
        m_intakeServo.stop();
    }


}

