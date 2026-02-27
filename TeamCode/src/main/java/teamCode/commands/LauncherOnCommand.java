package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
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

            m_velocityFromDistanceLUT.put(0.53, 1400.0);
            m_velocityFromDistanceLUT.put(0.60, 1500.0);
            m_velocityFromDistanceLUT.put(0.77, 1600.0);
            m_velocityFromDistanceLUT.put(0.87, 1700.0); //HOOD UP
            m_velocityFromDistanceLUT.put(1.13, 1800.0);
            m_velocityFromDistanceLUT.put(1.32, 1900.0);
            m_velocityFromDistanceLUT.put(1.66, 2000.0);
            m_velocityFromDistanceLUT.put(1.75, 2100.0);
            m_velocityFromDistanceLUT.put(1.87, 2200.0);
            m_velocityFromDistanceLUT.put(2.15, 2300.0);
            m_velocityFromDistanceLUT.put(2.51, 2400.0);
            m_velocityFromDistanceLUT.put(2.73, 2500.0);

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
                    double targetVelocity = getVelocityFromDistance(zMeters); // get velocity from LUT based on distance

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
