package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

/* PID to stop down when it gets close to centered
Hypothetically: If Left is Negative and Right is positive
if (HuskyLens Offset <= (Less than or Equal to) -0.15)
{
    Turn motor Left / negative
}
else if (HuskyLens Offset >= (Greater than or Equal to) 0.15)
{
    Turn motor Right / positive
}
*/

public class AimingTestingCommand extends CommandBase
{
    private final HuskyLensSubsystem m_huskySubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;
    // Assuming HuskyLens resolution is 320x240, center X is 160
    private static final int TARGET_CENTER_X = 160;
    private static final double KP = 0.01; // Proportional gain (needs tuning)

//    private double error = 160 - tag4;
    private double deadband = 5.0; // Ignore errors smaller than 5 pixels
//    private final DcMotor m_turnTableMotor;


    public AimingTestingCommand(HuskyLensSubsystem huskyLensSubsystem, TurnTableSubsystem turnTableSubsystem)
    {
        m_huskySubsystem = huskyLensSubsystem;
        m_turnTableSubsystem = turnTableSubsystem;
//        m_turnTableMotor = turntableMotor;
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

        if (m_huskySubsystem.isTagDetected())
        {
            // 1. Get the X position of your specific learned tag (e.g., ID 1)
            int targetX = m_huskySubsystem.getTargetCenterX();
            // 2. Calculate the error (160 is the center of the 320x240 screen)
            double error = TARGET_CENTER_X - targetX;
            double correction = error * KP; // Proportional correction
            // 3. DEADBAND LOGIC: If the error is small, do nothing
            double deadband = 5.0; // Adjust this (5-10) until wiggling stops

//            if (correction > 0.5) correction = 0.5;
//            if (correction < -0.5) correction = -0.5;
            m_turnTableSubsystem.turnSpeed(correction);

            m_turnTableSubsystem.turnSpeed(error * KP);

            if (Math.abs(error) < deadband)
            {
                m_turnTableSubsystem.stop();
                return; // Exit execute early so no power is applied
            }
        }
        // 4. Calculate correction with Proportional Gain (KP)

    }
//    else
//
//    {
//        m_turnTableSubsystem.stop();
//    }
//}
////        if (m_huskySubsystem.isTagDetected())
////        {
////            int targetX = m_huskySubsystem.getTargetCenterX();
////            double error = TARGET_CENTER_X - targetX; // Calculate difference from center
////            double correction = error * KP; // Proportional correction
////
////            // Limit correction speed
////            if (correction > 0.5) correction = 0.5;
////            if (correction < -0.5) correction = -0.5;
////
////            m_turnTableSubsystem.turnSpeed(correction);
////        }
////
////        else
////        {
////            m_turnTableSubsystem.stop(); // Stop if no tag detected
//////            m_turnTableMotor.setPower(0);
////        }
////
//////        if (Math.abs(error) < deadband)
//////        {
//////            m_turnTableSubsystem.stop();
//////        }
//////        else
//////        {
//////            m_turnTableSubsystem.turnSpeed(error * KP);
//////        }


    @Override
    public boolean isFinished()
    {
        // Finish when centered (within a small tolerance)
//        if (m_huskySubsystem.isTagDetected())
//        {
//            int targetX = m_huskySubsystem.getTargetCenterX();
//            return Math.abs(TARGET_CENTER_X - targetX) < 10; // Tolerance of 10 pixels (tune as needed)
//        }
        return false; // Continue indefinitely until centered or interrupted
    }

    @Override
    public void end(boolean interrupted)
    {
        m_turnTableSubsystem.stop();
    }
}
