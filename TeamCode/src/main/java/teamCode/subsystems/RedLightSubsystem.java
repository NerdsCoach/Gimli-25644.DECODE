package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Servo;

public class RedLightSubsystem extends SubsystemBase
{
    private GamepadEx m_gamepad1;
    private GamepadEx m_gamepad2;
//    public double timer;
    private final Servo m_light;



    public RedLightSubsystem(GamepadEx gamepad1, GamepadEx gamepad2, Servo light)
    {
        this.m_gamepad1 = gamepad1;
        this.m_gamepad2 = gamepad2;
        this.m_light = light;

    }

    public void inEndGame(double timer)
    {
//        if(timer > 89.5 && timer < 91 )
        if(timer > 136 && timer < 142) // TODO: changed timing to last 20 seconds

        {
            this.m_gamepad1.gamepad.rumble(500);
            this.m_gamepad2.gamepad.rumble(500);
        }
    }


//    public void redLight(double lightOn, double timer)
//    {
////        if(timer > 120 && timer < 125 )
//            if(timer > 130 && timer < 140 )
//            {
//            m_light.setPosition(lightOn);
//            }
//    }
}
