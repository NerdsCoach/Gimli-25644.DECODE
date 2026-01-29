package teamCode.Auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.DriveToPoint;
import teamCode.Pose2DUnNormalized;
import teamCode.commands.TimerCommand;

@Disabled
//@Autonomous(name="Red Move Auto", group="Pinpoint")
//@Disabled

public class RedMoveAuto extends LinearOpMode
{

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private GoBildaPinpointDriver m_odo;
    private TimerCommand m_timerCommand;

    private StateMachine m_stateMachine;

    private final ElapsedTime holdTimer = new ElapsedTime();
    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

    static final Pose2DUnNormalized Park = new Pose2DUnNormalized(DistanceUnit.MM, 400, 200, UnnormalizedAngleUnit.DEGREES, 0);

    enum StateMachine
    {
        WAITING_FOR_START,
        DRIVE_OFF_LINE,
        PARK,
        PARKED,
    }

    @Override
    public void runOpMode()
    {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        m_odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        m_odo.setOffsets(90, 5, DistanceUnit.MM);//these are tuned for Gimli 3110-0002-0001 Product Insight #1
        m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        m_odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, UnnormalizedAngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine m_stateMachine;
        m_stateMachine = StateMachine.WAITING_FOR_START;
//        sortPos = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", m_odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", m_odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", m_odo.getDeviceVersion());
        telemetry.addData("Device Scalar", m_odo.getYawScalar());
        telemetry.update();




        waitForStart();
        resetRuntime();

        while (opModeIsActive())
        {
            m_odo.update();

            switch (m_stateMachine) {
                case WAITING_FOR_START:

                    //the first step in the autonomous
                    m_stateMachine = StateMachine.DRIVE_OFF_LINE;
                    break;

                case DRIVE_OFF_LINE:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            Park, 0.5, 0)) {
                        telemetry.addLine("Done");

                    }
                    break;


            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:",m_stateMachine);

//            Pose2DUnNormalized pos = m_odo.getUnNormalizedPosition();

//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(UnnormalizedAngleUnit.DEGREES));
//            telemetry.addData("Position", data);

            telemetry.update();

        }
    }
}   // end class

