package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class HoodServoSubsystem extends SubsystemBase
{
    private final Servo m_aimingServo;

    // Fudge variable
    private double tuningPosition = 0.5; // Start in the exact middle
    // Your 7-9 points: {Distance from AprilTag, Servo Position}
    private final double[][] hoodLUT =
            //[] Means you're using an array, a set list of data
    {
            {0.53, 0.65},
            {0.80, 0.43},
            {0.99, 0.39},
            {1.07, 0.39},
            {1.20, 0.35},
            {1.35, 0.29},
            {1.49, 0.25},
            {1.79, 0.23},
            {2.05, 0.16},
            {2.36, 0.16}
    };

    public HoodServoSubsystem(HardwareMap hMap, String aimingServo)
    {
        this.m_aimingServo = hMap.get(Servo.class, aimingServo);
    }

    // The complete math method that replaces the broken placeholder line
    public double getTargetFromDistance(double currentDistance) {
        // Edge case: closer than your closest recorded data point
        if (currentDistance <= hoodLUT[0][0]) {
            return hoodLUT[0][1];
        }

        // Edge case: further than your furthest recorded data point
        if (currentDistance >= hoodLUT[hoodLUT.length - 1][0]) {
            return hoodLUT[hoodLUT.length - 1][1];
        }

        // Loop through the table to find which two points the robot is between
        for (int i = 0; i < hoodLUT.length - 1; i++) {
            if (currentDistance >= hoodLUT[i][0] && currentDistance <= hoodLUT[i+1][0]) {
                double d0 = hoodLUT[i][0];   // Lower distance bound
                double p0 = hoodLUT[i][1];   // Lower servo position
                double d1 = hoodLUT[i+1][0]; // Upper distance bound
                double p1 = hoodLUT[i+1][1]; // Upper servo position

                // Actual Linear Interpolation formula:
                double interpolatedValue = p0 + ((currentDistance - d0) / (d1 - d0)) * (p1 - p0);

                return interpolatedValue;
            }
        }

        // Ultimate fallback just in case something breaks down
        return hoodLUT[0][1];
    }

    public void setRawPosition(double position) {
        this.m_aimingServo.setPosition(position);
    }




    public void nudgeHoodUp()
    {
        tuningPosition += 0.01;
        // Range.clip(value, min, max) completely replaces the messy Math.max/min lines
//        tuningPosition = Range.clip(tuningPosition, 0.0, 1.0);
        tuningPosition = Range.clip(tuningPosition, 0.16, 0.65);

        this.m_aimingServo.setPosition(tuningPosition);
    }

    public void nudgeHoodDown()
    {
        tuningPosition -= 0.01;
        tuningPosition = Range.clip(tuningPosition, 0.16, 0.65);
        this.m_aimingServo.setPosition(tuningPosition);
    }

    // 3. A method so the command (or TeleOp) can read the fake position to display it
    public double getTuningPosition()
    {
        return tuningPosition;
    }

    public void pivotHood(double movePos)
    {
        this.m_aimingServo.setPosition(movePos);
    }
}