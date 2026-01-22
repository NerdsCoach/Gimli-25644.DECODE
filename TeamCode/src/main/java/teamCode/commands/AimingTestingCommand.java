package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class AimingTestingCommand extends CommandBase
{
    private final HuskyLensSubsystem m_huskySubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;
    private static final int TARGET_CENTER_X = 160;
    private static final double KP = 0.01; // Proportional gain (needs tuning)
    private double deadband = 5.0; // Ignore errors smaller than 5 pixels

    public AimingTestingCommand(HuskyLensSubsystem huskyLensSubsystem, TurnTableSubsystem turnTableSubsystem)
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

        if (m_huskySubsystem.isTagDetected())
        {
            // 1. Get the X position of your specific learned tag (e.g., ID 1)
            int targetX = m_huskySubsystem.getTargetCenterX();
            // 2. Calculate the error (160 is the center of the 320x240 screen)
            double error = TARGET_CENTER_X - targetX;
            double correction = error * KP; // Proportional correction
            // 3. DEADBAND LOGIC: If the error is small, do nothing
            double deadband = 5.0; // Adjust this (5-10) until wiggling stops

            m_turnTableSubsystem.turnSpeed(correction);

            m_turnTableSubsystem.turnSpeed(error * KP);

            if (Math.abs(error) < deadband)
            {
                m_turnTableSubsystem.stop();
            }
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_turnTableSubsystem.stop();
    }
}
