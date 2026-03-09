package teamCode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class PoseStorage
{
    public static int ySpecScore;
    public static double xEncoder;
    public static double yEncoder;
    public static double yReading = yEncoder;
    public static double odoHeading;

    public static Pose2D zero = new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES,0.0);
    public static Pose2DUnNormalized currentPose = new Pose2DUnNormalized(DistanceUnit.MM,xEncoder,yReading,UnnormalizedAngleUnit.DEGREES,odoHeading);
}