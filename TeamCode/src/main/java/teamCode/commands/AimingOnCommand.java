package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class AimingOnCommand extends CommandBase
{
    private final HuskyLensSubsystem m_huskySubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;
    // Assuming HuskyLens resolution is 320x240, center X is 160
    private static final int TARGET_CENTER_X = 160;
    private static final double KP = 0.004; //0.003 – 0.01	The multiplier that converts pixels to power.

    public AimingOnCommand(HuskyLensSubsystem huskyLensSubsystem, TurnTableSubsystem turnTableSubsystem)
    {
        m_huskySubsystem = huskyLensSubsystem;
        m_turnTableSubsystem = turnTableSubsystem;
        addRequirements(huskyLensSubsystem, turnTableSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        int targetX = m_huskySubsystem.getTargetCenterX();
        double error = TARGET_CENTER_X - targetX;
        double correction = error * KP;
        double deadband = 10.0;

        int currentPosition = m_turnTableSubsystem.getCurrentPosition();

        // 1. Limit correction speed (Clamping)
        if (correction > 0.3) correction = 0.3;
        if (correction < -0.3) correction = -0.3;

        if (m_huskySubsystem.isTagDetected())
        {
            if (Math.abs(error) < deadband)
            {
                m_turnTableSubsystem.stop();
            }
            else
            {
                // 2. Updated Directional Logic (Inclusive Limits)
                boolean canMoveNegative = (currentPosition > -850);
                boolean canMovePositive = (currentPosition < 850);

                if (correction > 0 && canMovePositive)
                {
                    m_turnTableSubsystem.turnSpeed(correction);
                }
                else if (correction < 0 && canMoveNegative)
                {
                    m_turnTableSubsystem.turnSpeed(correction);
                }
                else
                {
                    // We are at a limit and the HuskyLens wants to go further
                    m_turnTableSubsystem.stop();
                }
            }
        }
        else
        {
            m_turnTableSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished()
    {
        return false; // Continue indefinitely until centered or interrupted
    }

    @Override
    public void end(boolean interrupted)
    {
        m_turnTableSubsystem.stop();
    }

}
