package teamCode;

public class Constants
{
    public static final class DriveTrainConstants
    {
        public static final int kRandomValue = 0;
    }
    //Call values with
//    Constants.DriveTrainConstants.kRandomValue

    public static final class LiftArmConstants
    {
        public static final int kLiftArmCloseSample = 52;//2
        public static final int kLiftArmFarSample = 460;//450
        public static final int kLiftArmHighBasket = 2400;
        public static final int kLiftArmHighChamber = 2400;
        public static final int kLiftArmHome = 0;
        public static final int kLiftArmLowBasket = 2400;
        public static final int kLiftArmLowChamber = 675;
        public static final int kLiftArmScoreSpecimen = 200;
        public static final int kLiftArmFudgeFactorUp = 50;
        public static final int kLiftArmFudgeFactorDown = -50;
        public static final int kLiftArmIntakeReset = 550;
        public static final int kLiftArmReleaseClimbArm = 2400;
        public static final int kLiftArmSubSlide = 12;
        public static final int kLiftArmSubClose = 325;
    }

    public static final class SlideArmConstants
    {
        public static final int kSlideArmCloseSample = 40;
        public static final int kSlideArmFarSample = 1150;//434
        public static final int kSlideArmHighBasket = 1533;
        public static final int kSlideArmHighChamber = 394; //600
        public static final int kSlideArmHome = 40;
        public static final int kSlideArmLowBasket = 180; //200
        public static final int kSlideFudgeIn = -66;
        public static final int kSlideFudgeOut = 66;
        public static final int kSlideFudgeOutMax = 1451;
        public static final int kSlideSpecimenScore = 143;
        public static final int kSlideAutoScore = 40;
        public static final int kSLideArmSubClose = 317;
    }

    public static final class PivotIntakeConstants
    {
        public static final double kIntakePivotScore = 0.6;
        public static final double kIntakePivotSpecimen = 0.7;
        public static final double kIntakePivotPickUp = 0.54;
        public static final double kIntakePivotFarSample= 0.45;
    }
}
