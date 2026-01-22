package teamCode.subsystems;
//change 3 spots
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.function.DoubleSupplier;

import teamCode.DriveToPoint;
import teamCode.GoBildaPinpointReader;
import teamCode.Pose2DUnNormalized;

public class DriveSubsystem extends SubsystemBase
{

    public MecanumDrive m_drive;
    private DcMotor m_leftFront;
    private DcMotor m_rightFront;
    private DcMotor m_leftBack;
    private DcMotor m_rightBack;

    private GoBildaPinpointDriver m_odo;

    DriveToPoint nav = new DriveToPoint();

    private double m_lastRecordedAngle;
    private double m_currentAngle;
    private double error;


    public DriveSubsystem(MecanumDrive drive, GoBildaPinpointDriver odo)
    {
        this.m_drive = drive;
        this.m_currentAngle = 0.0;
        this.m_odo = odo;
        this.m_odo.setOffsets(87,-170, DistanceUnit.MM);
        this.m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.m_odo.setPosition(new Pose2D(DistanceUnit.MM, 0,0, AngleUnit.DEGREES, 0.0));

    }

    public void headingDrive(double leftX, double leftY, double rightX, double rightY)
    {
        m_drive.driveFieldCentric
                (
                        leftX * leftX * leftX * -1.0,
                        leftY * leftY * leftY * -1.0,//-1
                        getJoystickAngle(rightX, rightY),
                        m_odo.getHeading(AngleUnit.DEGREES)  //try this with robot, or change DEGREES to RADIANS below, and check for all the places we getHeading to make the same changes

//                        Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES))  //different from last year
                );
        m_odo.update();
    }

    public void autoHeadingDrive (DoubleSupplier targetX, DoubleSupplier targetY, DoubleSupplier targetAngle)
    {
        m_drive.driveFieldCentric
                (
                        this.getDeltaPosition(targetX.getAsDouble())[0],
                        this.getDeltaPosition(targetY.getAsDouble())[1],
                        getTurnPower(true,0),
                        m_odo.getHeading(AngleUnit.DEGREES)  //try this with robot, or change DEGREES to RADIANS below, and check for all the places we getHeading to make the same changes
//                        Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES))//different from last year
                );
    }

    public double getJoystickAngle (double rightX, double rightY) /* angle in degrees*/
    {
        return getTurnPower(rightX > 0.5 || rightX < -0.5 || rightY > 0.5 || rightY < -0.5, Math.atan2(rightX, rightY * -1) * -1 * (180 / Math.PI));
    }


    public double getTurnPower(boolean deadband, double angle)
    {
        turnTo(deadband, angle);
//        System.out.println("Running!");

        if (Math.abs(error) > 6)
        {
            double motorPower = 0.5;//.5 for normal//0.4 is best for stop
            error = error - getAngle();
            return motorPower * error / 100 + (0.1 * (error / Math.abs(error)));
        }
        else
        {
            return 0.0;
        }
    }

    public void turnTo(boolean deadband, double angle)
    {
//        double orientation = Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES));
        double orientation = m_odo.getHeading(AngleUnit.DEGREES);
        m_odo.update();
        double desiredAngle;
        if (deadband)
        {
            desiredAngle = angle;
        }
        else
        {
            desiredAngle = orientation;
        }

        error = desiredAngle - orientation;

        if(error > 180)
        {
            error -= 360;
        }
        else if (error < -180)
        {
            error += 360;
        }

        turn(error, desiredAngle);
    }

    public void turn(double degrees, double desiredAngle)
    {
        resetAngle();

        error = degrees;
    }
    public void resetAngle()
    {
        m_odo.update();
//        m_lastRecordedAngle = Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES));
        m_lastRecordedAngle = m_odo.getHeading(AngleUnit.DEGREES);
        m_currentAngle = 0;
    }

    public double getAngle()
    {
//        double orientation = Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES));
        double orientation = m_odo.getHeading(AngleUnit.DEGREES);
        double deltaAngle = orientation - m_lastRecordedAngle;

        if (deltaAngle > 180)
        {
            deltaAngle -= 360;
        }
        else if (deltaAngle <= -180)
        {
            deltaAngle += 360;
        }

        m_odo.update();
        m_currentAngle += deltaAngle;
        m_lastRecordedAngle = orientation;
        return m_currentAngle;
    }
    public void resetOdo()
    {
        this.m_odo.resetPosAndIMU();
    }

    public Pose2DUnNormalized getUnNormalizedPosition()
    {
        return new Pose2DUnNormalized(DistanceUnit.MM,
                m_odo.getPosX(DistanceUnit.MM),
                m_odo.getPosY(DistanceUnit.MM),
                UnnormalizedAngleUnit.DEGREES,
                m_odo.getHeading(UnnormalizedAngleUnit.DEGREES));
    }

    public void updateOdo()
    {
        this.m_odo.update();
    }

    public double[] getDeltaPosition(double target)
    {
        return new double[]
                {
                        (target - m_odo.getPosX(DistanceUnit.MM)) / target,
                        (target - m_odo.getPosY(DistanceUnit.MM)) / target
                };
    }
    public void autoSetPower()
    {
        //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
        this.m_leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
        this.m_rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
        this.m_leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
        this.m_rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
        System.out.println("Power" + nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
    }
//    public MecanumDrive m_drive;
//    private DcMotor m_leftFront;
//    private DcMotor m_rightFront;
//    private DcMotor m_leftBack;
//    private DcMotor m_rightBack;
//
//    private GoBildaPinpointDriver m_odo;
//
//    DriveToPoint nav = new DriveToPoint();
//
//    private double m_lastRecordedAngle;
//    private double m_currentAngle;
//    private double error;
//
//
//    public DriveSubsystem(MecanumDrive drive, GoBildaPinpointDriver odo)
//    {
//        this.m_drive = drive;
//        this.m_currentAngle = 0.0;
//        this.m_odo = odo;
//        this.m_odo.setOffsets(87,-170, DistanceUnit.MM);
//        this.m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        this.m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        this.m_odo.setPosition(new Pose2D(DistanceUnit.MM, 0,0, AngleUnit.DEGREES, 0.0));
//
//    }
//
//    public void headingDrive(double leftX, double leftY, double rightX, double rightY)
//    {
//        m_drive.driveFieldCentric
//                (
//                        leftX * leftX * leftX * -1.0,
//                        leftY * leftY * leftY * -1.0,//-1
//                        getJoystickAngle(rightX, rightY),
//                        m_odo.getHeading(AngleUnit.DEGREES)  //try this with robot, or change DEGREES to RADIANS below, and check for all the places we getHeading to make the same changes
//
////                        Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES))  //different from last year
//                );
//        m_odo.update();
//    }
//
//    public void autoHeadingDrive (DoubleSupplier targetX, DoubleSupplier targetY, DoubleSupplier targetAngle)
//    {
//        m_drive.driveFieldCentric
//                (
//                        this.getDeltaPosition(targetX.getAsDouble())[0],
//                        this.getDeltaPosition(targetY.getAsDouble())[1],
//                        getTurnPower(true,0),
//                        m_odo.getHeading(AngleUnit.DEGREES)  //try this with robot, or change DEGREES to RADIANS below, and check for all the places we getHeading to make the same changes
////                        Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES))//different from last year
//                );
//    }
//
//    public double getJoystickAngle (double rightX, double rightY) /* angle in degrees*/
//    {
//        return getTurnPower(rightX > 0.5 || rightX < -0.5 || rightY > 0.5 || rightY < -0.5, Math.atan2(rightX, rightY * -1) * -1 * (180 / Math.PI));
//    }
//
//
//    public double getTurnPower(boolean deadband, double angle)
//    {
//        turnTo(deadband, angle);
////        System.out.println("Running!");
//
//        if (Math.abs(error) > 6)
//        {
//            double motorPower = 0.5;//.5 for normal//0.4 is best for stop
//            error = error - getAngle();
//            return motorPower * error / 100 + (0.1 * (error / Math.abs(error)));
//        }
//        else
//        {
//            return 0.0;
//        }
//    }
//
//    public void turnTo(boolean deadband, double angle)
//    {
////        double orientation = Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES));
//        double orientation = m_odo.getHeading(AngleUnit.DEGREES);
//        m_odo.update();
//        double desiredAngle;
//        if (deadband)
//        {
//            desiredAngle = angle;
//        }
//        else
//        {
//            desiredAngle = orientation;
//        }
//
//        error = desiredAngle - orientation;
//
//        if(error > 180)
//        {
//            error -= 360;
//        }
//        else if (error < -180)
//        {
//            error += 360;
//        }
//
//        turn(error, desiredAngle);
//    }
//
//    public void turn(double degrees, double desiredAngle)
//    {
//        resetAngle();
//
//        error = degrees;
//    }
//    public void resetAngle()
//    {
//        m_odo.update();
////        m_lastRecordedAngle = Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES));
//        m_lastRecordedAngle = m_odo.getHeading(AngleUnit.DEGREES);
//        m_currentAngle = 0;
//    }
//
//    public double getAngle()
//    {
////        double orientation = Math.toDegrees(m_odo.getHeading(AngleUnit.DEGREES));
//        double orientation = m_odo.getHeading(AngleUnit.DEGREES);
//        double deltaAngle = orientation - m_lastRecordedAngle;
//
//        if (deltaAngle > 180)
//        {
//            deltaAngle -= 360;
//        }
//        else if (deltaAngle <= -180)
//        {
//            deltaAngle += 360;
//        }
//
//        m_odo.update();
//        m_currentAngle += deltaAngle;
//        m_lastRecordedAngle = orientation;
//        return m_currentAngle;
//    }
//    public void resetOdo()
//    {
//        this.m_odo.resetPosAndIMU();
//    }
//
//    public Pose2DUnNormalized getUnNormalizedPosition()
//    {
//        return new Pose2DUnNormalized(DistanceUnit.MM,
//                m_odo.getPosX(DistanceUnit.MM),
//                m_odo.getPosY(DistanceUnit.MM),
//                UnnormalizedAngleUnit.DEGREES,
//                m_odo.getHeading(UnnormalizedAngleUnit.DEGREES));
//    }
//
//    public void updateOdo()
//    {
//        this.m_odo.update();
//    }
//
//    public double[] getDeltaPosition(double target)
//    {
//        return new double[]
//                {
//                        (target - m_odo.getPosX(DistanceUnit.MM)) / target,
//                        (target - m_odo.getPosY(DistanceUnit.MM)) / target
//                };
//    }
//    public void autoSetPower()
//    {
//        //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
//        this.m_leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
//        this.m_rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
//        this.m_leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
//        this.m_rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
//        System.out.println("Power" + nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
//    }
}