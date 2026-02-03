package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.rev.RevTouchSensor;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.LimitSwitchSubsystem;


public class TransferLimitCommand extends CommandBase
{
    private final LimitSwitchSubsystem m_limitSwitchSubsystem;
    private boolean lastState = false;

    public TransferLimitCommand(LimitSwitchSubsystem limitSwitch)
    {
        this.m_limitSwitchSubsystem = limitSwitch;
        addRequirements(this.m_limitSwitchSubsystem);
    }

    @Override
    public void initialize()
    {
        // This allows the command to "run again" by starting from zero
        m_limitSwitchSubsystem.resetHits();
        lastState = m_limitSwitchSubsystem.isPressed();
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

        m_limitSwitchSubsystem.setTransferPower(-0.34);//-0.25 ,-0.27
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
    }
}
