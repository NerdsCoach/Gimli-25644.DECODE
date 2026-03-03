package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.TreeMap;
import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LimeLightSubsystem;

public class LauncherOnCommand extends CommandBase
    {
        private final LauncherSubsystem m_launcherSubsystem;
        private final AxeSubsystem m_axeSubsystem;
        private final HoodServoSubsystem m_hoodSubsystem;
        private final LimeLightSubsystem m_limelightSubsystem;

        private final InterpLUT m_velocityLUT = new InterpLUT();//TODO test this
        private static final double m_aimFar = Constants.AimingConstants.kFarAim;
        private static final double m_aimClose = Constants.AimingConstants.kCloseAim;
        private static final double m_axeDown = Constants.AxeConstants.kAxeDown;

        private final TreeMap<Double, Double> m_velocityFromDistanceLUT = new TreeMap<>();
        private double m_lastKnownSpeed;

        public LauncherOnCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem,
                                 HoodServoSubsystem hoodSubsystem, LimeLightSubsystem limelightSubsystem)
        {

            this.m_launcherSubsystem = launcherSubsystem;
            this.m_axeSubsystem = axeSubsystem;
            this.m_hoodSubsystem = hoodSubsystem;
            this.m_limelightSubsystem = limelightSubsystem;
            m_lastKnownSpeed = 1500.0;

            addRequirements(this.m_launcherSubsystem, this.m_axeSubsystem, this.m_hoodSubsystem);

            // This table maps the distance in meters (from the Limelight) to the required launcher velocity.

//            InterpLUT lut = new InterpLUT();//TODO test this

            m_velocityLUT.add(0.49, 1450.0);
            m_velocityLUT.add(0.65, 1550.0);
            m_velocityLUT.add(0.85, 1650.0);
            m_velocityLUT.add(0.99, 1750.0);
            m_velocityLUT.add(1.12, 1850.0);
            m_velocityLUT.add(1.28, 1950.0);
            m_velocityLUT.add(1.36, 2050.0);
            m_velocityLUT.add(1.61, 2150.0);
            m_velocityLUT.add(1.93, 2250.0); // hood up
            m_velocityLUT.add(1.95, 2260.0);
            m_velocityLUT.add(2.14, 2300.0);
            m_velocityLUT.add(2.22, 2350.0);
            m_velocityLUT.add(2.38, 2450.0);
            m_velocityLUT.add(2.42, 2550.0);
// ... add other points ...
            m_velocityLUT.createLUT();
        }

        /**
         * Calculates the target launcher velocity based on the distance to the target.
         * It uses linear interpolation between the known points in the lookup table.
         *
         * @param distanceMeters The distance to the target in meters.
         * @return The calculated target velocity for the launcher.
         */
        public double getVelocityFromDistance(double distanceMeters)
        {
            Double lowKey = m_velocityFromDistanceLUT.floorKey(distanceMeters);
            Double highKey = m_velocityFromDistanceLUT.ceilingKey(distanceMeters);

            // for distances outside the range of the LUT
            if (lowKey == null)
            {
                return m_velocityFromDistanceLUT.get(highKey);
            }
            if (highKey == null)
            {
                return m_velocityFromDistanceLUT.get(lowKey);
            }
            if (lowKey.equals(highKey))
            {
                return m_velocityFromDistanceLUT.get(lowKey);
            }

            // Perform linear interpolation
            double lowVelocity = m_velocityFromDistanceLUT.get(lowKey);
            double highVelocity = m_velocityFromDistanceLUT.get(highKey);
            return lowVelocity + (distanceMeters - lowKey) * (highVelocity - lowVelocity) / (highKey - lowKey);
        }

        @Override
        public void execute()
        {
            LLResult result = m_limelightSubsystem.getLatestResult();
            this.m_axeSubsystem.pivotAxe(m_axeDown);

            // Can we see the correct April Tag?
            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())
            {
                LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
                Pose3D pose = tag.getTargetPoseCameraSpace();

                if (pose != null)
                {
                    double zMeters = Math.abs(pose.getPosition().z); // Distance (Z) in meters
//                    double targetVelocity = getVelocityFromDistance(zMeters); // get velocity from LUT based on distance
                    double targetVelocity = m_velocityLUT.get(zMeters); //TODO test this

                    m_launcherSubsystem.setMotorVelocity(targetVelocity);
                    m_lastKnownSpeed = targetVelocity;

                    // Hood
                    if (targetVelocity > 1700)
                    {
                        this.m_hoodSubsystem.pivotHood(m_aimFar);
                    }
                    else
                    {
                        this.m_hoodSubsystem.pivotHood(m_aimClose);
                    }
                }
            }
            else
            {
                // IF WE LOSE SIGHT: Use the memory variable!
                m_launcherSubsystem.setMotorVelocity(m_lastKnownSpeed);
//                this.m_hoodSubsystem.pivotHood(m_aimClose);
            }
        }

        @Override
        public void end(boolean interrupted)
        {
        }

        @Override
        public boolean isFinished()
        { return false; }
    }
