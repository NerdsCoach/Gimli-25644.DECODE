package teamCode.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.TreeMap;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.LauncherSubsystem;

public class LauncherOnCommand extends CommandBase
{
    private final LauncherSubsystem m_launcherSubsystem;
    private final AxeSubsystem m_axeSubsystem;
    private final HuskyLensSubsystem m_huskySubsystem;
    private final HoodServoSubsystem m_hoodSubsystem;
    private static final double m_aimFar = Constants.AimingConstants.kFarAim;
    private static final double m_aimClose = Constants.AimingConstants.kCloseAim;


    private TreeMap<Double, Double> distanceLUT = new TreeMap<>();
    private TreeMap<Double, Double> velocityLUT = new TreeMap<>();

    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private int m_position;
    private static final int m_up = 1;
    private static final int m_down = 0;
    private double m_lastKnownSpeed;

    public LauncherOnCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem, HuskyLensSubsystem huskyLensSubsystem, HoodServoSubsystem hoodSubsystem)
    {
        m_position = m_down;

        this.m_launcherSubsystem = launcherSubsystem;
        this.m_huskySubsystem = huskyLensSubsystem;
        this.m_axeSubsystem = axeSubsystem;
        this.m_hoodSubsystem = hoodSubsystem;

        m_lastKnownSpeed = 1500.0; // Set a safe default starting speed

        addRequirements(this.m_launcherSubsystem, this.m_axeSubsystem, this.m_hoodSubsystem);

//        velocityLUT.put(84.0, 2400.0); // 84 inches -> 2400 ticks/sec
//        velocityLUT.put(72.0, 2300.0); // 72 inches -> 2300 ticks/sec
//        velocityLUT.put(60.0, 2100.0); // 60 inches -> 2100 ticks/sec
//        velocityLUT.put(48.0, 2000.0); // 48 inches -> 2000 ticks/sec
//        velocityLUT.put(36.0, 1960.0); // 36 inches -> 1960 ticks/sec
//        velocityLUT.put(30.0, 1870.0); // 30 inches -> 1900 ticks/sec
//        velocityLUT.put(24.0, 1800.0); // 30 inches -> 1900 ticks/

        velocityLUT.put(24.0, 1800.0); // inches -> ticks/sec
        velocityLUT.put(30.0, 1800.0); // inches -> ticks/sec
        velocityLUT.put(36.0, 1870.0); // inches -> ticks/sec
        velocityLUT.put(42.0, 1960.0); // inches -> ticks/sec
        velocityLUT.put(48.0, 2000.0); // inches -> ticks/sec
        velocityLUT.put(53.0, 2050.0); // inches -> ticks/sec
        velocityLUT.put(67.0, 2150.0); // inches -> ticks/sec
        velocityLUT.put(75.0, 2250.0); // inches -> ticks/sec
        velocityLUT.put(81.0, 2350.0); // inches -> ticks/sec
        velocityLUT.put(94.0, 2400.0); // inches -> ticks/sec
        velocityLUT.put(102.0, 2450.0); // inches -> ticks/sec


        // Add your real-world measurements here:
        distanceLUT.put(74.0, 24.0); // 74 pixels = 24 in (7ft)
        distanceLUT.put(62.0, 30.0); // 62 pixels = 30 inches (6ft)
        distanceLUT.put(25.0, 36.0); // 25 pixels = 36 inches (5ft)
        distanceLUT.put(46.0, 42.0); // 46 pixels = 42 inches (4ft)
        distanceLUT.put(40.0, 48.0); // 48 pixels = 40 inches (3ft)
        distanceLUT.put(38.0, 53.0); // 53 pixels = 38 inches (2.5ft)
        distanceLUT.put(30.0, 67.0); // 67 pixels = 30 inches (2ft)
        distanceLUT.put(28.0, 75.0); // 75 pixels = 28 inches (2ft)
        distanceLUT.put(26.0, 81.0); // 75 pixels = 28 inches (2ft)
        distanceLUT.put(22.0, 94.0); // 75 pixels = 28 inches (2ft)
        distanceLUT.put(20.0, 102.0); // 75 pixels = 28 inches (2ft)

    }
    public double getDistance(double currentWidth)
    {
        Double lowKey = distanceLUT.floorKey(currentWidth);
        Double highKey = distanceLUT.ceilingKey(currentWidth);
        if (lowKey == null) return distanceLUT.get(highKey);
        if (highKey == null) return distanceLUT.get(lowKey);
        if (lowKey.equals(highKey)) return distanceLUT.get(lowKey);
        return distanceLUT.get(lowKey) + (currentWidth - lowKey) *
                (distanceLUT.get(highKey) - distanceLUT.get(lowKey)) / (highKey - lowKey);
    }

    public double getSubTargetVelocity(double distance)
    {
        Double lowKey = velocityLUT.floorKey(distance);
        Double highKey = velocityLUT.ceilingKey(distance);
        if (lowKey == null) return velocityLUT.get(highKey);
        if (highKey == null) return velocityLUT.get(lowKey);
        if (lowKey.equals(highKey)) return velocityLUT.get(lowKey);
        return velocityLUT.get(lowKey) + (distance - lowKey) *
                (velocityLUT.get(highKey) - velocityLUT.get(lowKey)) / (highKey - lowKey);
    }


    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {

        double width = m_huskySubsystem.getTargetWidth();
        int centerX = m_huskySubsystem.getTargetCenterX();

            this.m_axeSubsystem.pivotAxe(m_axeDown);
            if (width > 0)
            {
                // 1. HANDLE SPEED (Distance Logic)
                double distance = getDistance(width);
                double targetVelocity = getSubTargetVelocity(distance);
                m_launcherSubsystem.setMotorVelocity(targetVelocity);
                // 2. SAVE this to your memory variable
                m_lastKnownSpeed = targetVelocity;

                // 3. Set the motor
                m_launcherSubsystem.setMotorVelocity(targetVelocity);

                if(targetVelocity > 2000)
                {
                    this.m_hoodSubsystem.pivotHood(m_aimFar);
                } else if (targetVelocity < 2000)
                {
                    this.m_hoodSubsystem.pivotHood(m_aimClose);
                }
            }
            else
            {
                // If we lose sight, DON'T set to 0. Use the saved memory!
                m_launcherSubsystem.setMotorVelocity(m_lastKnownSpeed);
            }

    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
