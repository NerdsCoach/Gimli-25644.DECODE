package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class AimingOffCommand extends CommandBase
{
    private final HuskyLensSubsystem m_huskySubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;
    private final ColorSensorSubsystem m_colorSubsystem;

    public AimingOffCommand(HuskyLensSubsystem huskyLensSubsystem, TurnTableSubsystem turnTableSubsystem, ColorSensorSubsystem colorSensorSubsystem)
    {
        m_huskySubsystem = huskyLensSubsystem;
        m_turnTableSubsystem = turnTableSubsystem;
        m_colorSubsystem = colorSensorSubsystem;
        addRequirements(huskyLensSubsystem, turnTableSubsystem, colorSensorSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        m_turnTableSubsystem.stop(); // Stop if no tag detected
        m_colorSubsystem.setLEDOff();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_turnTableSubsystem.stop();
    }
}