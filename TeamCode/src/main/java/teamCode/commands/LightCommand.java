package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.SorterServoSubsystem;

public class LightCommand extends CommandBase
{
    private final LightSubsystem m_lightSubsystem;

    public LightCommand(LightSubsystem light)
    {
        this.m_lightSubsystem = light;
        addRequirements(this.m_lightSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_lightSubsystem.on(0.722);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean interrupted)
    {
    }
}
