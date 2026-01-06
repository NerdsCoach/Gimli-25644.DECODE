package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitchSubsystem extends SubsystemBase
{
    private DigitalChannel m_limitSwitch;

    public LimitSwitchSubsystem(DigitalChannel limitSwitch)
    {
        // Initialize the limit switch from the hardware map
        m_limitSwitch = limitSwitch;

        // Set to INPUT mode so it can read data
        m_limitSwitch.setMode(DigitalChannel.Mode.INPUT);

    }

    public boolean isPressed()
    {
        // Many limit switches are 'active low', meaning they return FALSE when pressed.
        // Check your specific wiring; often !m_limitSwitch.getState() is needed.
        return !m_limitSwitch.getState();
    }

//    public void setPower(double power) {
//        // Safety check: Don't allow movement in the restricted direction if pressed
//        if (isPressed() && power > 0) {
//            motor.setPower(0);
//        } else {
//            motor.setPower(power);
//        }
//    }
}
