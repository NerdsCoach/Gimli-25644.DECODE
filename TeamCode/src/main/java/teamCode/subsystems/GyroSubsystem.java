package teamCode.subsystems;

import static teamCode.PoseStorage.odoHeading;
import static teamCode.PoseStorage.xEncoder;
import static teamCode.PoseStorage.yEncoder;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.Pose2DUnNormalized;
import teamCode.PoseStorage;

public class GyroSubsystem extends SubsystemBase
{
    private GoBildaPinpointDriver m_odo;

    public GyroSubsystem(GoBildaPinpointDriver odo)
    {
       this.m_odo = odo;
    }

    public void resetGyro()
    {
        this.m_odo.setPosition(PoseStorage.zero);
    }

}
