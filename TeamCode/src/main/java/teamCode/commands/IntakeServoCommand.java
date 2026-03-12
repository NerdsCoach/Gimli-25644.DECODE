package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakeMotorSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;

public class IntakeServoCommand extends CommandBase
{
    private final IntakeServoSubsystem m_intakeServoSubsystem;
    private boolean lastState = false;

    public IntakeServoCommand(IntakeServoSubsystem intakeServo)
    {
        this.m_intakeServoSubsystem = intakeServo;
        addRequirements(this.m_intakeServoSubsystem);
    }

    @Override
    public void initialize()
    {
        m_intakeServoSubsystem.resetHits();
        lastState = m_intakeServoSubsystem.isPressed();
    }

    @Override
    public void execute()
    {
        boolean currentState = m_intakeServoSubsystem.isPressed();

        // Only count if it transitions from FALSE to TRUE
        if (currentState && !lastState)
        {
            m_intakeServoSubsystem.incrementHits();
        }
        lastState = currentState;

        m_intakeServoSubsystem.spinServo(0.2); //-0.29
    }
    @Override
    public void end(boolean interrupted)
    {
        m_intakeServoSubsystem.spinServo(0.0);
    }

    @Override
    public boolean isFinished()
    {
        return m_intakeServoSubsystem.getHitCount() >= 1 ;
    }
}
