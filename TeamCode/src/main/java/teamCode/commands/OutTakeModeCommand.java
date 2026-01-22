package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.SorterServoSubsystem;

public class OutTakeModeCommand extends CommandBase
{
    private final IntakeServoSubsystem m_intakeWheelSubsystem;
    private static final double m_intakeOn = Constants.IntakeConstants.kOutTake;
    private static final double m_intakeOff = Constants.IntakeConstants.kIntakeOff;
    private int m_position;
    private static final int  m_off = 1;
    private static final int  m_on = 0;

    public OutTakeModeCommand(IntakeServoSubsystem intake)
    {
        this.m_intakeWheelSubsystem = intake;
        addRequirements(this.m_intakeWheelSubsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        if (m_position == m_off)
        {
            this.m_intakeWheelSubsystem.spinIntake(m_intakeOn);
            m_position = m_on;
        }
        else if (m_position == m_on)
        {
            this.m_intakeWheelSubsystem.spinIntake(m_intakeOff);
            m_position = m_off;
        }
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
