package teamCode.commands;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.GamepadSubsystem;
//TODO Making it work with color and the timer

public class TimerCommand extends CommandBase
{
    private GamepadSubsystem m_gamepadSubsystem;
    public DoubleSupplier m_timer;

    public TimerCommand(GamepadSubsystem gamepadSubsystem, DoubleSupplier timer)
    {
        this.m_gamepadSubsystem = gamepadSubsystem;
        this.m_timer = timer;
        addRequirements(gamepadSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_gamepadSubsystem.inEndGame(m_timer.getAsDouble());
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
