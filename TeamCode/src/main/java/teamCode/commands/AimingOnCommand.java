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
//    private static final double KP = 0.01; // Proportional gain (needs tuning)
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
        // Code to run when command starts
    }

    @Override
    public void execute()
    {
        //TODO GOOGLE 2 SOLUTION
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

        //TODO GOOGLE 1
//        int targetX = m_huskySubsystem.getTargetCenterX();
//        double error = TARGET_CENTER_X - targetX;
//        double correction = error * KP;
//        double deadband = 10.0;
//
//        // 1. Get current encoder positions from your turntable
//        int currentPosition = m_turnTableSubsystem.getCurrentPosition();
//
//        // 2. Limit correction speed
//        if (correction > 0.3) correction = 0.3;
//        if (correction < -0.3) correction = -0.3;
//
//        if (m_huskySubsystem.isTagDetected()) {
//            if (Math.abs(error) < deadband) {
//                m_turnTableSubsystem.stop();
//            } else {
//                // 3. Directional Limit Logic:
//                // Only allow movement if it's NOT moving toward a reached limit
//                boolean canMoveNegative = (currentPosition > -850);
//                boolean canMovePositive = (currentPosition < 850);
//
//                if (correction > 0 && canMovePositive) {
//                    m_turnTableSubsystem.turnSpeed(correction); // Move positive if not at max
//                } else if (correction < 0 && canMoveNegative) {
//                    m_turnTableSubsystem.turnSpeed(correction); // Move negative if not at min
//                } else {
//                    m_turnTableSubsystem.stop(); // Stop only if trying to go PAST the limit
//                }
//            }
//        } else {
//            m_turnTableSubsystem.stop();
//        }
    }

//    public void execute()
//    { int targetX = m_huskySubsystem.getTargetCenterX();
//        double error = TARGET_CENTER_X - targetX; // Calculate difference from center
//        double correction = error * KP; // Proportional correction
//        double deadband = 10.0; // Adjust this (Started @ 5-10) until wiggling stops 8!
//
//        if (m_huskySubsystem.isTagDetected())
//        // && m_turnTableSubsystem.atTargetLeft(-850) && m_turnTableSubsystem.atTargetRight(850)
//        {
//            if (Math.abs(error) < deadband)
//            {
//                m_turnTableSubsystem.stop();
//            }
//            else if (Math.abs(error) > deadband);
//            {
//                // Limit correction speed
//                if (correction > 0.3) correction = 0.3;
//                if (correction < -0.3) correction = -0.3; // 0.5
//                m_turnTableSubsystem.turnSpeed(correction);
//            }
//        }
//        else
//        {
//            m_turnTableSubsystem.stop(); // Stop if no tag detected
//        }
//    }

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
