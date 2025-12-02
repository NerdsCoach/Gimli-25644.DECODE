package teamCode.Auto;

import static teamCode.Constants.SorterConstants.kScorePos1;
import static teamCode.Constants.SorterConstants.kScorePos2;
import static teamCode.Constants.SorterConstants.kScorePos3;
import static teamCode.Constants.TransferConstants.kTransferDown;
import static teamCode.Constants.TransferConstants.kTransferUp;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamCode.Constants;
import teamCode.DriveToPoint;
import teamCode.Pose2DUnNormalized;
import teamCode.commands.TimerCommand;
import teamCode.subsystems.GyroSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.LauncherMotorSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TransferServoSubsystem;


@Autonomous(name="Red Score Auto", group="Pinpoint")
//@Disabled

public class RedScoreAuto extends LinearOpMode
{

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private CRServo m_intakeServo;
    private Servo m_transferServo;
    private Servo m_sorterServo;
    private DcMotor m_launcherMotorRed;

    private GyroSubsystem m_gyroSubsystem;
    private LauncherMotorSubsystem m_launcherMotorSubsystem;
    private TransferServoSubsystem m_transferServoSubsystem;
    private SorterServoSubsystem m_sorterServoSubsystem;
    private IntakeServoSubsystem m_intakeServoSubsystem;

    private final ElapsedTime holdTimer = new ElapsedTime();
    private GoBildaPinpointDriver m_odo;
    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

    enum StateMachine
    {
        WAITING_FOR_START,
        DRIVE_TO_SCORE,
        SCORE_1,
        SCORE_1_RESET,
        SCORE_2,
        SCORE_2_RESET,
        SCORE_3,
        SCORE_3_RESET,
        TRANSFER_DOWN,
        TURN_OFF,
        PARK,
        PARKED,
    }
    static final Pose2DUnNormalized ScorePosition = new Pose2DUnNormalized(DistanceUnit.MM, 100, 100, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized Park = new Pose2DUnNormalized(DistanceUnit.MM, 354, 443, UnnormalizedAngleUnit.DEGREES, 0);

//    private TimerCommand m_timerCommand;

//    private static final double m_scorePos1 = Constants.SorterConstants.kScorePos1;
//    private static final double m_scorePos2 = Constants.SorterConstants.kScorePos2;
//    private static final double m_scorePos3 = Constants.SorterConstants.kScorePos3;

//    private StateMachine m_stateMachine;

//    public int sortPos = 1;





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
        m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        m_odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, UnnormalizedAngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;
//        sortPos = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", m_odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", m_odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", m_odo.getDeviceVersion());
        telemetry.addData("Device Scalar", m_odo.getYawScalar());
        telemetry.update();


        this.m_launcherMotorRed = hardwareMap.get(DcMotor.class, "launcherMotorRed");

        this.m_intakeServo = new CRServo(hardwareMap, "intakeServo");
        this.m_sorterServoSubsystem = new SorterServoSubsystem(hardwareMap, "m_sorterServo" );
        this.m_transferServoSubsystem = new TransferServoSubsystem(hardwareMap, "transferServo");

        this.m_launcherMotorSubsystem = new LauncherMotorSubsystem(this.m_launcherMotorRed/*, this.m_launcherMotorBlue*/);

        this.m_intakeServoSubsystem = new IntakeServoSubsystem(this.m_intakeServo);
//        this.m_sorterServo = hardwareMap.get(ServoImplEx.class, "sorterServo");
//        this.m_transferServo = hardwareMap.get(Servo.class, "transferServo");

        waitForStart();
        resetRuntime();

        while (opModeIsActive())
        {
            m_odo.update();

            switch (stateMachine)
            {
                case WAITING_FOR_START:

                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_SCORE;
                    break;

                case DRIVE_TO_SCORE:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */

                    holdTimer.reset();
                    this.m_sorterServoSubsystem.sort(kScorePos1);
                    this.m_launcherMotorSubsystem.launch();
                    telemetry.addLine("Launch Motor On");
                    stateMachine = StateMachine.SCORE_1_RESET;

                    break;


                case SCORE_1_RESET:
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            ScorePosition, 0.5, 2.0))
                    {
                        this.m_transferServoSubsystem.transfer(1.0);
                        telemetry.addLine("Transfer Up");
                        holdTimer.reset();
                        sleep(5000);

                        stateMachine = StateMachine.SCORE_2;
                    }

                    break;


                case SCORE_2:
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            ScorePosition, 0.5, 2.0))
                {
                    this.m_transferServoSubsystem.transfer(0);
                    this.m_launcherMotorSubsystem.stop();
                    telemetry.addLine("Transfer Down");

                    holdTimer.reset();
                    stateMachine = StateMachine.SCORE_2_RESET;
                }
                    break;


//                    this.m_sorterServoSubsystem.sort(kScorePos2);
//                    if (this.m_sorterServoSubsystem.atTarget(kScorePos2))
//                    {
//                        this.m_transferServoSubsystem.transfer(kTransferUp);
//                        stateMachine = StateMachine.SCORE_2_RESET;
//                    }
//                    break;
//
                case SCORE_2_RESET:
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            ScorePosition, 0.5, 2.0))
                    {
                        telemetry.addLine("End");

                    }
//
//                    if (this.m_transferServoSubsystem.atTarget(kTransferUp))
//                    {
//                        this.m_transferServoSubsystem.transfer(kTransferDown);
//                        stateMachine = StateMachine.SCORE_3;
//                    }
                    break;
//
//
//                case SCORE_3:
//                    this.m_sorterServoSubsystem.sort(kScorePos3);
//                    if (this.m_sorterServoSubsystem.atTarget(kScorePos3))
//                    {
//                        this.m_transferServoSubsystem.transfer(kTransferUp);
//                        stateMachine = StateMachine.SCORE_3_RESET;
//                    }
//
//                    break;
//
//                case SCORE_3_RESET:
//
//                    if (this.m_transferServoSubsystem.atTarget(kTransferUp))
//                    {
//                        this.m_transferServoSubsystem.transfer(kTransferDown);
//                        stateMachine = StateMachine.TURN_OFF;
//                    }
//                    break;


//                case TURN_OFF:
//                    this.m_launcherMotorSubsystem.stop();
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            0.6, .75))
//                        {
//                            stateMachine = StateMachine.PARK;
//                        }
//                    break;
//
//                case PARK:
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM),m_odo.getPosY(DistanceUnit.MM),UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            Park, 0.5, 0))
//                    {
//                        telemetry.addLine("Parked!");
//                        stateMachine = StateMachine.PARKED;
//                    }
//                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:",stateMachine);

//            Pose2DUnNormalized pos = m_odo.getUnNormalizedPosition();

//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(UnnormalizedAngleUnit.DEGREES));
//            telemetry.addData("Position", data);

            telemetry.update();

        }
    }
}   // end class

