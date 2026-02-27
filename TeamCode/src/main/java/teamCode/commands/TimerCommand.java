package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import teamCode.subsystems.GamepadSubsystem;

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
