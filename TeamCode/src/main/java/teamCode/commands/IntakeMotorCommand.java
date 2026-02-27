package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakeMotorSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;

public class IntakeMotorCommand extends CommandBase
{
    private final IntakeMotorSubsystem m_intakeMotorSubsystem;
    private final IntakeServoSubsystem m_intakeServoSubsystem;
    private static final double m_intakeOn = Constants.IntakeConstants.kIntakeOn;
    private static final double m_intakeOff = Constants.IntakeConstants.kIntakeOff;
    private int m_position;
    private static final int  m_off = 1;
    private static final int  m_on = 0;

    public IntakeMotorCommand(IntakeMotorSubsystem intakeMotor, IntakeServoSubsystem intakeServo)
    {
        this.m_intakeMotorSubsystem = intakeMotor;
        this.m_intakeServoSubsystem = intakeServo;
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
            this.m_intakeServoSubsystem.spinServo(m_intakeOn);
            m_position = m_on;
        }
        else if (m_position == m_on)
        {
            this.m_intakeMotorSubsystem.spinMotorIntake(m_intakeOff);
            this.m_intakeServoSubsystem.spinServo(m_intakeOff);
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
