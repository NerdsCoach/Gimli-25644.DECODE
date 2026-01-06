package teamCode.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.HoodServoSubsystem;

public class HoodServoFarCommand extends CommandBase
{
    private static final double m_aimFar = Constants.LauncherServoConstants.kAimFar;
    private final HoodServoSubsystem m_hoodServoSubsystem;
    private int m_position;
    private static final int m_down = 0;

    public HoodServoFarCommand(HoodServoSubsystem servoSubsystem)
    {
        this.m_hoodServoSubsystem = servoSubsystem;
        m_position = m_down;
        addRequirements(this.m_hoodServoSubsystem);
    }

    @Override
    public void initialize()
    {
    }
    @Override
    public void execute()
    {
            this.m_hoodServoSubsystem.pivotHood(m_aimFar);
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
