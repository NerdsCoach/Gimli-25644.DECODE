package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.DriveSubsystem;
import teamCode.subsystems.LimitSwitchSubsystem;

public class TransferLimitCommand extends CommandBase
{
    private final LimitSwitchSubsystem m_limitSwitchSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    private boolean lastState = false;

    public TransferLimitCommand(LimitSwitchSubsystem limitSwitch, DriveSubsystem driveSubsystem)
    {
        this.m_limitSwitchSubsystem = limitSwitch;
        this.m_driveSubsystem = driveSubsystem;
        addRequirements(this.m_limitSwitchSubsystem, this.m_driveSubsystem);
    }

    @Override
    public void initialize()
    {
        // This allows the command to "run again" by starting from zero
        m_limitSwitchSubsystem.resetHits();
        lastState = m_limitSwitchSubsystem.isPressed();
        m_driveSubsystem.setSpeedModifier(0.1); // Drops drive speed to 10%
        System.out.println("Launcher ON - Sniper Mode Active!");

    }

    @Override
    public void execute()
    {
        boolean currentState = m_limitSwitchSubsystem.isPressed();

        // Only count if it transitions from FALSE to TRUE
        if (currentState && !lastState)
        {
            m_limitSwitchSubsystem.incrementHits();
        }
        lastState = currentState;

        m_limitSwitchSubsystem.setTransferPower(-0.5); //-0.29
    }

    @Override
    public boolean isFinished()
    {
        return m_limitSwitchSubsystem.getHitCount() >= 1 ;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_limitSwitchSubsystem.setTransferPower(0.0);
        m_driveSubsystem.setSpeedModifier(1.0); // Restores full drive speed
        System.out.println("Launcher OFF - Sniper Mode Disabled!");
    }
}
