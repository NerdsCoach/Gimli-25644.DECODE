package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.Constants;
import teamCode.subsystems.HoodServoSubsystem;

public class HoodUpCommand extends CommandBase
{
    private static final double m_aimFar = Constants.AimingConstants.kFarAim;
    private static final double m_aimClose = Constants.AimingConstants.kCloseAim;
    private final HoodServoSubsystem m_hoodServoSubsystem;

    public HoodUpCommand(HoodServoSubsystem hoodServoSubsystem)
    {
        this.m_hoodServoSubsystem = hoodServoSubsystem;
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
        this.m_hoodServoSubsystem.pivotHood(m_aimClose);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
