package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class GamepadSubsystem extends SubsystemBase
{
    private GamepadEx m_gamepad1;
    private GamepadEx m_gamepad2;




    public GamepadSubsystem(GamepadEx gamepad1, GamepadEx gamepad2)
    {
        this.m_gamepad1 = gamepad1;
        this.m_gamepad2 = gamepad2;

    }

    public void inEndGame(double timer)
    {
//        if(timer > 89.5 && timer < 91 )
        if(timer > 100 && timer < 106) // TODO: changed timing to last 20 seconds

        {
            this.m_gamepad1.gamepad.rumble(500);
            this.m_gamepad2.gamepad.rumble(500);
        }
    }


}
