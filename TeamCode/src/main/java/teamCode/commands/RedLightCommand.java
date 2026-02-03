package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.GamepadSubsystem;
import teamCode.subsystems.LightSubsystem;
//TODO Making it work with color and the timer

public class RedLightCommand extends CommandBase
{
    private GamepadSubsystem m_gamepadSubsystem;
    private LightSubsystem m_lightSubsystem;
    public DoubleSupplier m_timer;

    public RedLightCommand(GamepadSubsystem gamepadSubsystem, DoubleSupplier timer, LightSubsystem redLight)
    {
        this.m_gamepadSubsystem = gamepadSubsystem;
        this.m_timer = timer;
        this.m_lightSubsystem = redLight;
    }

    @Override
    public void initialize()
    {
    }

//    @Override
//    public void execute()
//    {
//
//        if(this.m_gamepadSubsystem.redLight(m_timer.getAsDouble()))
//        {
//                this.m_lightSubsystem.on(0.277);
//        }
//    }

    @Override
    public void execute() {
        if (this.m_gamepadSubsystem.redLight(m_timer.getAsDouble()))
        {
            this.m_lightSubsystem.on(0.277);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
