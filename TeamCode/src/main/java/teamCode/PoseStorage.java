package teamCode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class PoseStorage
{
//    public static class poseStorage
//    {
//

        public static Pose2D zero = new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES,0.0);
//        public static Pose2DUnNormalized posi = new Pose2DUnNormalized(DistanceUnit.MM,0,0,UnnormalizedAngleUnit.DEGREES,0);
        public static double xEncoder;
        public static double yEncoder;
        public static double yReading = yEncoder;
        public static double odoHeading;
        public static Pose2DUnNormalized basketAutoEnd = new Pose2DUnNormalized(DistanceUnit.MM, 1270, -205,UnnormalizedAngleUnit.DEGREES,90.5);
        public static Pose2DUnNormalized chamberAutoEnd = new Pose2DUnNormalized(DistanceUnit.MM, 73, -1087,UnnormalizedAngleUnit.DEGREES,-91.6);

        public static Pose2DUnNormalized currentPose = new Pose2DUnNormalized(DistanceUnit.MM,xEncoder,yReading,UnnormalizedAngleUnit.DEGREES,odoHeading);

        public static int ySpecScore;
    }
//}
