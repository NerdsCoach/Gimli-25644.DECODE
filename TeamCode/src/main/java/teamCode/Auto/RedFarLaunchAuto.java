package teamCode.Auto;



import static teamCode.Constants.AxeConstants.kAxeDown;
import static teamCode.Constants.AxeConstants.kAxeUp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.DriveToPoint;
import teamCode.Pose2DUnNormalized;
import teamCode.commands.TimerCommand;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.ColorSensorSubsystem;
//import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TransferSubsystem;
import teamCode.subsystems.TurnTableSubsystem;


@Autonomous(name="Red Far Launch Auto", group="Pinpoint")
//@Disabled

public class RedFarLaunchAuto extends LinearOpMode
{
    /* Drivetrain */
    private MecanumDrive m_drive;

    /* PID */
    private PIDController m_pIDController;
    //Motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx m_launcherMotor;
    public DcMotor m_turnTableMotor;

    //Servos
    private Servo m_AxeServo;
    private CRServo m_transferServo;
    private CRServo m_sorterServo;
//
    //Sensors
    private GoBildaPinpointDriver m_odo;
//    public HuskyLens m_huskylens;
    public NormalizedColorSensor m_colorSensor;
    private ElapsedTime m_StateTime = new ElapsedTime();

    //
    //Subsystems
    private AxeSubsystem m_axeSubsystem;
    private SorterServoSubsystem m_sorterServoSubsystem;
    private ColorSensorSubsystem m_colorSensorSubsystem;
    private LightSubsystem m_lightSubsystem;
    private LauncherSubsystem m_launcherSubsystem;
    private TransferSubsystem m_transferSubsystem;
    private TurnTableSubsystem m_turnTableSubsystem;
//
////    private HuskyLensSubsystem m_huskyLensSubsystem;


    private TimerCommand m_timerCommand;
    private StateMachine m_stateMachine;

    private final ElapsedTime holdTimer = new ElapsedTime();
    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

    static final Pose2DUnNormalized Start = new Pose2DUnNormalized(DistanceUnit.MM, 0, 0, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized Park = new Pose2DUnNormalized(DistanceUnit.MM, 400, 200, UnnormalizedAngleUnit.DEGREES, 0);

    enum StateMachine
    {
        WAITING_FOR_START,
        PREPARE_FOR_BATTLE,
        LAUNCH_BALL_1,
        WAIT_1,
        LAUNCH_BALL_2,
        WAIT_2,
        LAUNCH_BALL_3,
        WAIT_3,
        WAIT_4,
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
        m_odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

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

        this.m_sorterServo = new CRServo(hardwareMap, "sorterServo");
        this.m_transferServo = new CRServo(hardwareMap, "transferServo");


        this.m_turnTableMotor = hardwareMap.get(DcMotor.class, "turnTableMotor");
        this.m_launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotorRed");
        this.m_axeSubsystem = new AxeSubsystem(hardwareMap, "axeServo");
        this.m_lightSubsystem = new LightSubsystem(hardwareMap, "light");

        this.m_sorterServoSubsystem = new SorterServoSubsystem(this.m_sorterServo);
        this.m_transferSubsystem = new TransferSubsystem(this.m_transferServo);
        this.m_turnTableSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);

        this.m_launcherSubsystem = new LauncherSubsystem(this.m_launcherMotor);
        this.m_colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

//        this.m_huskyLensSubsystem = new HuskyLensSubsystem(this.m_huskylens);

        waitForStart();
        resetRuntime();

        while (opModeIsActive())
        {
            m_odo.update();

            switch (m_stateMachine)
            {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                m_stateMachine = StateMachine.PREPARE_FOR_BATTLE;
                    break;

                case PREPARE_FOR_BATTLE:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    holdTimer.reset();
                    // Sorter, Launcher, and Turntable on. Axe down
                    this.m_sorterServoSubsystem.spinSorter(-1.0);
                    this.m_launcherSubsystem.setMotorVelocity(1000);
                    this.m_turnTableSubsystem.Turn(500); //TODO: -650
                    this.m_axeSubsystem.pivotAxe(kAxeUp);


                    if (m_StateTime.time() > 2.0)
                    {
                        if(m_turnTableSubsystem.atTargetRight(500))
                        {
                            telemetry.addLine("Launch Motor On");
                            m_stateMachine = StateMachine.LAUNCH_BALL_1;
                        }
                    }

                    break;

                case LAUNCH_BALL_1:
                    if (this.m_turnTableSubsystem.atTargetRight(500))
                    {
                        //Transfer On
                        this.m_transferSubsystem.spinTransfer(-0.5);
                        telemetry.addLine("Transfer On");
                        m_StateTime.reset();
                        m_stateMachine = StateMachine.WAIT_1;
                    }
                    break;

                case WAIT_1:
                                    // Check if the desired time (e.g., 2.0 seconds) has passed
                    if (m_StateTime.time() > 2.0)
                    {
                        // Time is up, transition to the next state
                        this.m_transferSubsystem.spinTransfer(0);
                        telemetry.addLine("Done");
                        m_stateMachine = StateMachine.DRIVE_OFF_LINE;
                        // Optional: reset encoders or perform a one-time action
                    }

                    break;


//                case LAUNCH_BALL_2:
//                    if (this.m_turnTableSubsystem.atTargetRight(500))
//                    {
//                        //Transfer On
//                        this.m_transferSubsystem.spinTransfer(-0.7);
//                        telemetry.addLine("Transfer On");
//                        m_StateTime.reset();
//                        m_stateMachine = StateMachine.WAIT_2;
//                    }
//                    break;

//                case WAIT_2:
//                    // Check if the desired time (e.g., 2.0 seconds) has passed
//                    if (m_StateTime.time() > 2)
//                    {
//                        // Time is up, transition to the next state
//                        this.m_transferSubsystem.spinTransfer(0.0);
//                        telemetry.addLine("Done");
//                        m_stateMachine = StateMachine.LAUNCH_BALL_3;
//                        // Optional: reset encoders or perform a one-time action
//                    }
//                    break;
//
//                case LAUNCH_BALL_3:
//                    if (this.m_turnTableSubsystem.atTargetRight(500))
//                    {
//                        //Transfer On
//                        this.m_transferSubsystem.spinTransfer(-0.5);
//                        telemetry.addLine("Transfer On");
//                        m_StateTime.reset();
//                        m_stateMachine = StateMachine.WAIT_4;
//                    }
//                    break;
//
//                case WAIT_4:
//                    // Check if the desired time (e.g., 2.0 seconds) has passed
//                    if (m_StateTime.time() > 2)
//                    {
//                        // Time is up, transition to the next state
//                        this.m_transferSubsystem.spinTransfer(0.0);
//                        telemetry.addLine("Done");
//                        m_stateMachine = StateMachine.DRIVE_OFF_LINE;
//                        // Optional: reset encoders or perform a one-time action
//                    }
//                    break;

                case DRIVE_OFF_LINE:
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            Park, 0.5, 0))

                    //Move Off Line
                    holdTimer.reset();
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                                Start, 0.5, 2))
                    {
                        this.m_launcherSubsystem.stop();
                        this.m_axeSubsystem.pivotAxe(kAxeDown);
                        this.m_transferSubsystem.spinTransfer(0.0);
                        this.m_sorterServoSubsystem.spinSorter(0);
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

