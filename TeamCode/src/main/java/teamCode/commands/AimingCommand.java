package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.AimingServoSubsystem;
import teamCode.subsystems.TurnTableSubsystem;
import teamCode.subsystems.HuskyLensAimingSubsystem;

public class AimingCommand extends CommandBase
{
    public HuskyLensAimingSubsystem m_huskyLensSubsystem;
    public AimingServoSubsystem m_aimingServoSubsystem;
    public TurnTableSubsystem m_turnTableSubsystem;

    // Constants for PID-like control
    private final double kP = 0.01; // Proportional gain
    private final double DEADZONE = 5; // Pixels in center of screen

    public AimingCommand(HuskyLensAimingSubsystem huskyLens, AimingServoSubsystem launcher)
    {
        this.m_huskyLensSubsystem = huskyLens;
        this.m_aimingServoSubsystem = launcher;
        addRequirements(huskyLens, launcher);
    }

    @Override
    public void execute()
    {
        if (m_huskyLensSubsystem.isTargetDetected())
        {
            double targetCenterX = m_huskyLensSubsystem.getTargetCenterX();
            // Assuming HuskyLens screen width is 320 pixels
            double screenCenter = 160;

            double error = targetCenterX - screenCenter;

            if (Math.abs(error) > DEADZONE)
            {
                // Adjust servo position based on error
                double panAdjustment = error * kP;
                double currentPanPosition = m_aimingServoSubsystem.getPanPosition();
                double newPanPosition = currentPanPosition - panAdjustment;

                // Clamp the new position to stay within servo's range
                newPanPosition = Math.max(AimingServoSubsystem.PAN_MIN, Math.min(AimingServoSubsystem.PAN_MAX, newPanPosition));

                m_aimingServoSubsystem.setPanPosition(newPanPosition);
            }
        }
    }
    @Override
    public boolean isFinished()
    {
        // The command will run until interrupted.
        return false;
    }
}
