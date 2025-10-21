package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.DriveToPoint;
import teamCode.Pose2DUnNormalized;
import teamCode.PoseStorage;
import teamCode.subsystems.DriveSubsystem;
import teamCode.subsystems.GyroSubsystem;

//@Disabled
public class Pose2DSubmersibleCommand extends CommandBase
{
         //Initializing Motors, Subsystems, Java Programs
        private DcMotor m_leftFront;
        private DcMotor m_rightFront;
        private DcMotor m_leftBack;
        private DcMotor m_rightBack;
        public DriveSubsystem m_driveSubsystem;
        public GyroSubsystem m_gyroSubsystem;


    public GoBildaPinpointDriver m_odo; // Declare OpMode member for the Odometry Computer
        DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

//        public static int ySpecScore;
        public int specimen = 0;

    public Pose2DSubmersibleCommand(DriveSubsystem driveSubsystem, GoBildaPinpointDriver odo,
                                    DcMotor LEFT_FRONT, DcMotor RIGHT_FRONT, DcMotor LEFT_BACK, DcMotor RIGHT_BACK)

        {
            this.m_driveSubsystem = driveSubsystem;
            this.m_odo = odo;
            this.m_leftFront = LEFT_FRONT;
            this.m_rightFront = RIGHT_FRONT;
            this.m_leftBack = LEFT_BACK;
            this.m_rightBack = RIGHT_BACK;

            addRequirements(m_driveSubsystem, m_gyroSubsystem);
        }


    @Override
    public void initialize()
    {
    }
    @Override
    public void execute()
    {
        this.m_leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT)*-1);
        this.m_rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
        this.m_leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK)*-1);
        this.m_rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
//nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.


        this.m_driveSubsystem.updateOdo();


    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished() {
        if
        (this.nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
            new Pose2DUnNormalized (DistanceUnit.MM, 630, PoseStorage.ySpecScore, UnnormalizedAngleUnit.DEGREES, -180),0.6, 0))
        {
//            ySpecScore = ySpecScore + 30;
            return true;
        }
        return false;
    }

}
