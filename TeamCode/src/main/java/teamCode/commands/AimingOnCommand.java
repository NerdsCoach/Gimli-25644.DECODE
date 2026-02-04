package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.TreeMap;

import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class AimingOnCommand extends CommandBase
{
    private final HuskyLensSubsystem m_huskySubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;
    private final LightSubsystem m_lightSubsystem;
    // Assuming HuskyLens resolution is 320x240, center X is 160
    private static final int TARGET_CENTER_X = 160;
    private static final double KP = 0.0025; //0.003 – 0.01	The multiplier that converts pixels to power.

    private double lastError = 0;

    public AimingOnCommand(HuskyLensSubsystem huskyLensSubsystem, TurnTableSubsystem turnTableSubsystem, LightSubsystem lightSubsystem)
    {
        m_huskySubsystem = huskyLensSubsystem;
        m_turnTableSubsystem = turnTableSubsystem;
        m_lightSubsystem = lightSubsystem;
        addRequirements(huskyLensSubsystem, turnTableSubsystem, lightSubsystem);
    }

    @Override
    public void initialize()
    {
        m_lightSubsystem.on(.6);
    }

    @Override
    public void execute() {
        if (!m_huskySubsystem.isTagDetected()) {
            m_turnTableSubsystem.stop();
            return;
        }

        int targetX = m_huskySubsystem.getTargetCenterX();
        double error = TARGET_CENTER_X - targetX;
        double deadband = 10.0;

        // 1. DAMPING MATH
        // Lower KP to prevent overshooting, higher KD to act as a brake
        double kP = 0.002;
        double kD = 0.001;

        double errorChange = error - lastError;
        double correction = (error * kP) + (errorChange * kD);
        lastError = error;

        // 2. SMART POWER FLOOR
        // Only apply minPower if we are far enough away to need a "kick"
        double minPower = 0.08;
        if (Math.abs(error) > 15.0 && Math.abs(correction) < minPower) {
            correction = (error > 0) ? minPower : -minPower;
        }

        if (Math.abs(error) < deadband) {
            m_turnTableSubsystem.stop();
            return;
        }

        if (correction > 0.4) correction = 0.4;
        if (correction < -0.4) correction = -0.4;

        int currentPosition = m_turnTableSubsystem.getCurrentPosition();
        boolean canMoveNegative = (currentPosition > -850);
        boolean canMovePositive = (currentPosition < 850);

        if (correction > 0 && canMovePositive) {
            m_turnTableSubsystem.turnSpeed(correction);
        } else if (correction < 0 && canMoveNegative) {
            m_turnTableSubsystem.turnSpeed(correction);
        } else {
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
        m_lightSubsystem.off(0.0);
    }
}
