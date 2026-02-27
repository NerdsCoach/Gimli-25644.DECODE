package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakeMotorSubsystem;

public class OutTakeMotorCommand extends CommandBase
{
    private final IntakeMotorSubsystem m_intakeMotorSubsystem;
    private static final double m_intakeOn = Constants.IntakeConstants.kOutTake;
    private static final double m_intakeOff = Constants.IntakeConstants.kIntakeOff;
    private int m_position;
    private static final int  m_off = 1;
    private static final int  m_on = 0;

    public OutTakeMotorCommand(IntakeMotorSubsystem intakeMotor)
    {
        this.m_intakeMotorSubsystem = intakeMotor;
        addRequirements(this.m_intakeMotorSubsystem);
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
            this.m_intakeMotorSubsystem.spinMotorIntake(m_intakeOn);
            m_position = m_on;
        }
        else if (m_position == m_on)
        {
            this.m_intakeMotorSubsystem.spinMotorIntake(m_intakeOff);
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
