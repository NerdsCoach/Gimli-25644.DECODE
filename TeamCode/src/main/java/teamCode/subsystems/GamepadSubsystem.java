package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadSubsystem extends SubsystemBase
{
    private GamepadEx m_gamepad1;
    private GamepadEx m_gamepad2;
    public double timer;


    public GamepadSubsystem(GamepadEx gamepad1, GamepadEx gamepad2)
    {
        this.m_gamepad1 = gamepad1;
        this.m_gamepad2 = gamepad2;
    }


    public void inEndGame(double timer)
    {
        if(timer > 85.0 && timer < 86.5 )
        {
            this.m_gamepad1.gamepad.rumble(1000);
            this.m_gamepad2.gamepad.rumble(1000);
        }
    }
    public void speedManatee()
    {
        this.m_gamepad1.gamepad.rumbleBlips(3);
//        this.m_gamepad1.gamepad.setLedColor(1.0,0.2,0.0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    public void speedTurbo()
    {
        this.m_gamepad1.gamepad.rumbleBlips(1);

//        this.m_gamepad1.gamepad.setLedColor(0.0,0.0,1.0,Gamepad.LED_DURATION_CONTINUOUS);
    }
}
