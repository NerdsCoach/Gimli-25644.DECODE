package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

/* PID to slow down when it gets close to centered
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

public class AimingCommand extends CommandBase
{
    private final HuskyLensSubsystem m_huskySubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;
    // Assuming HuskyLens resolution is 320x240, center X is 160
    private static final int TARGET_CENTER_X = 160;
    private static final double KP = 0.01; // Proportional gain (needs tuning)
//    private final DcMotor m_turnTableMotor;

    public AimingCommand(HuskyLensSubsystem huskyLensSubsystem, TurnTableSubsystem turnTableSubsystem)
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
            int targetX = m_huskySubsystem.getTargetCenterX();
            double error = TARGET_CENTER_X - targetX; // Calculate difference from center
            double correction = error * KP; // Proportional correction

            // Limit correction speed
            if (correction > 0.5) correction = 0.5;
            if (correction < -0.5) correction = -0.5;

            m_turnTableSubsystem.turn(correction);
        }
        else
        {
            m_turnTableSubsystem.stop(); // Stop if no tag detected
//            m_turnTableMotor.setPower(0);
        }
    }

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

//    private final HuskyLensSubsystem m_huskySubsystem;
//    private final int m_targetTagId;
//    private final double Kp = 0.005;
//    private final int CENTER_X = 160;
////    private final HuskyLensAimingSubsystem m_huskyLensSubsystem;
////    private final TurnTableSubsystem m_turnTableSubsystem;
//////    private final AimingServoSubsystem m_aimingServoSubsystem;
////    private final int centerX = 160; // HuskyLens X resolution is 320 pixels
////    private final double kP = 0.01; // Proportional control constant
//    public AimingCommand(HuskyLensSubsystem m_huskySubsystem, int targetTagId)
//    {
//        this.m_huskySubsystem = m_huskySubsystem;
//        this.m_targetTagId = targetTagId;
//        addRequirements(m_huskySubsystem);
////        this.m_huskyLensSubsystem = hls;
////        this.m_turnTableSubsystem = ttm;
////        addRequirements(hls, ttm);
//    }
//    @Override
//    public void initialize()
//    {
//        HuskyLens.Algorithm.TAG_RECOGNITION.ordinal();
//        this.m_huskySubsystem.AimAtTag(4);
//
//    }
//    @Override
//    public void execute()
//    {
////        HuskyLens.Algorithm.TAG_RECOGNITION.ordinal();
////        this.m_huskySubsystem.AimAtTag(4);
//        if (m_huskySubsystem.isTargetDetected())
//        {
//            int error = m_huskySubsystem.targetX - CENTER_X;
//            double motorPower = error * Kp;
//
//            // Constrain power to a safe range (e.g., -0.5 to 0.5)
////            motorPower = Math.max(-0.5, Math.min(0.5, motorPower));
//        }
//        else
//        {
//            m_huskySubsystem.Stop();
//        }
//    }
//
//    @Override
//    public void end(boolean interrupted)
//    {
////        m_turnTableSubsystem.stopMotor();
//    }
//
//    @Override
//    public boolean isFinished()
//    {
//        return true;
//        // Command finishes when target is roughly centered (e.g., error < 5 pixels)
////        return m_huskyLensSubsystem.targetDetected && Math.abs(m_huskyLensSubsystem.targetX - centerX) < 5;
//    }
}
//    public HuskyLensAimingSubsystem m_huskyLensSubsystem;
////    public AimingServoSubsystem m_aimingServoSubsystem;
//    public TurnTableSubsystem m_turnTableSubsystem;
//
//    // Constants for PID-like control
//    private final double kP = 0.01; // Proportional gain
//    private final double DEADZONE = 5; // Pixels in center of screen
//
//    public AimingCommand(HuskyLensAimingSubsystem huskyLens, TurnTableSubsystem turnTable )
//    {
//        this.m_huskyLensSubsystem = huskyLens;
////        this.m_aimingServoSubsystem = launcher;
//        addRequirements(huskyLens, turnTable);
//    }
//
//    @Override
//    public void execute()
//    {
//        if (m_huskyLensSubsystem.isTargetDetected())
//        {
//            double targetCenterX = m_huskyLensSubsystem.getTargetCenterX();
//            // Assuming HuskyLens screen width is 320 pixels
//            double screenCenter = 160;
//
//            double error = targetCenterX - screenCenter;
//
//            if (Math.abs(error) > DEADZONE)
//            {
//                // Adjust servo position based on error
//                double panAdjustment = error * kP;
//                double currentPanPosition = m_turnTableSubsystem.getPanPosition();
//                double newPanPosition = currentPanPosition - panAdjustment;
//
//                // Clamp the new position to stay within servo's range
//                newPanPosition = Math.max(TurnTableSubsystem.PAN_MIN, Math.min(TurnTableSubsystem.PAN_MAX, newPanPosition));
//
//                m_turnTableSubsystem.setPanPosition();
////                m_aimingServoSubsystem.setPanPosition(newPanPosition);
//            }
////        if (m_huskyLensSubsystem.isTargetDetected())
////        {
////            double targetCenterX = m_huskyLensSubsystem.getTargetCenterX();
////            // Assuming HuskyLens screen width is 320 pixels
////            double screenCenter = 160;
////
////            double error = targetCenterX - screenCenter;
////
////            if (Math.abs(error) > DEADZONE)
////            {
////                // Adjust servo position based on error
////                double panAdjustment = error * kP;
////                double currentPanPosition = m_aimingServoSubsystem.getPanPosition();
////                double newPanPosition = currentPanPosition - panAdjustment;
////
////                // Clamp the new position to stay within servo's range
////                newPanPosition = Math.max(AimingServoSubsystem.PAN_MIN, Math.min(AimingServoSubsystem.PAN_MAX, newPanPosition));
////
////                m_aimingServoSubsystem.setPanPosition(newPanPosition);
////            }
//
//        }
//    }
//    @Override
//    public boolean isFinished()
//    {
//        // The command will run until interrupted.
//        return false;
//    }
//}
