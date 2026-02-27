package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.Constants;
import teamCode.subsystems.HoodServoSubsystem;

public class HoodDownCommand extends CommandBase
{
    private static final double m_aimClose = Constants.AimingConstants.kCloseAim;
    private final HoodServoSubsystem m_hoodServoSubsystem;
    private int m_position;
    private static final int m_up = 1;
    private static final int m_down = 0;

    public HoodDownCommand(HoodServoSubsystem servoSubsystem)
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
            this.m_hoodServoSubsystem.pivotHood(m_aimClose);
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
