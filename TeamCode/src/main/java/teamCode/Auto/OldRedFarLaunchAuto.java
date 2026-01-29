package teamCode.Auto;


import static teamCode.Constants.AxeConstants.kAxeDown;
import static teamCode.Constants.AxeConstants.kAxeUp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import teamCode.Constants;
import teamCode.DriveToPoint;
import teamCode.Pose2DUnNormalized;
import teamCode.commands.TimerCommand;
import teamCode.commands.TransferLimitCommand;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.ColorSensorSubsystem;
//import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.LimitSwitchSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
//import teamCode.subsystems.TransferSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

@Autonomous(name="Red Far Launch Auto", group="Pinpoint")
@Disabled

public class OldRedFarLaunchAuto extends LinearOpMode
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
    private Servo m_hoodServo;
    public CRServo m_intakeServo;


    //Sensors
    private GoBildaPinpointDriver m_odo;
    public NormalizedColorSensor m_colorSensor;
    private ElapsedTime m_StateTime = new ElapsedTime();
    private RevTouchSensor m_limitSwitch;

    //Subsystems
    private AxeSubsystem m_axeSubsystem;
    private SorterServoSubsystem m_sorterServoSubsystem;
    private ColorSensorSubsystem m_colorSensorSubsystem;
    private LightSubsystem m_lightSubsystem;
    private LauncherSubsystem m_launcherSubsystem;
    private TurnTableSubsystem m_turnTableSubsystem;
    private LimitSwitchSubsystem m_limitSwitchSubsystem;
    private HoodServoSubsystem m_hoodServoSubsystem;
    private IntakeServoSubsystem m_intakeServoSubsystem;


    //Commands
    private TransferLimitCommand m_transferLimitCommand;

    private TimerCommand m_timerCommand;
    private StateMachine m_stateMachine;

    private boolean lastState = false;
    public int count = 0;



    //Driving
    private final ElapsedTime holdTimer = new ElapsedTime();
    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

    static final Pose2DUnNormalized Start = new Pose2DUnNormalized(DistanceUnit.MM, 0, 0, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized Move = new Pose2DUnNormalized(DistanceUnit.MM, 0, 200, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized StartPickUp = new Pose2DUnNormalized(DistanceUnit.MM, 320, 610, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized EndPickUp = new Pose2DUnNormalized(DistanceUnit.MM, 1100, 610, UnnormalizedAngleUnit.DEGREES, 0);
    static final Pose2DUnNormalized Park = new Pose2DUnNormalized(DistanceUnit.MM, 400, 200, UnnormalizedAngleUnit.DEGREES, 0);

    private static final double m_aimFar = Constants.AimingConstants.kFarAim;

    enum StateMachine
    {
        WAITING_FOR_START,
        PREPARE_FOR_BATTLE,
        LAUNCH_BALL_1,
        WAIT_1,
        COUNTER_SCORE,
        LAUNCH_BALL_2,
        WAIT_2,
        LAUNCH_BALL_3,
        WAIT_3,
        LAUNCH_BALL_4,
        WAIT_4,
        START_PICK_UP,
        PICK_UP,
        PARKED,
    }

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        //Driving Motors
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

        m_odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        m_odo.setOffsets(90, 5, DistanceUnit.MM);//these are tuned for Gimli 3110-0002-0001 Product Insight #1
        m_odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
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

        this.m_sorterServo = new CRServo(hardwareMap, "sorterServo");
        this.m_transferServo = new CRServo(hardwareMap, "transferServo");

        //Mechanism Motors
        this.m_turnTableMotor = hardwareMap.get(DcMotor.class, "turnTableMotor");
        this.m_launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotorRed");

        //Servos and Sensors
        this.m_axeSubsystem = new AxeSubsystem(hardwareMap, "axeServo");
        this.m_lightSubsystem = new LightSubsystem(hardwareMap, "light");
        this.m_limitSwitch = hardwareMap.get(RevTouchSensor.class, "limitSwitch");
//        this.m_transferServo = new CRServo(hardwareMap, "transferServo");
        this.m_hoodServoSubsystem = new HoodServoSubsystem(hardwareMap, "aimingServo");
        this.m_intakeServo = new CRServo(hardwareMap, "intakeServo");

        this.m_sorterServoSubsystem = new SorterServoSubsystem(this.m_sorterServo);
//        this.m_transferSubsystem = new TransferSubsystem(this.m_transferServo);

        this.m_turnTableSubsystem = new TurnTableSubsystem(this.m_turnTableMotor);
        this.m_limitSwitchSubsystem = new LimitSwitchSubsystem(this.m_limitSwitch, this.m_transferServo);
        this.m_transferLimitCommand = new TransferLimitCommand(this.m_limitSwitchSubsystem);
        this.m_intakeServoSubsystem = new IntakeServoSubsystem(this.m_intakeServo);

        this.m_launcherSubsystem = new LauncherSubsystem(this.m_launcherMotor);
        this.m_colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        waitForStart();
        resetRuntime();

        //DONE Launcher motors RPM is WAY too slow, speed it up
        //DONE Transfer is going based on seconds not limit switch, which is bad needs to be changed
        //TODO: only launches one ball, then lifts axe MAKE IT LAUNCH MULTIPLE AND KEEP AXE DOWN
        //TODO: Doesn't drive away, that needs to happen

        while (opModeIsActive())
        {
            m_odo.update();

            switch (m_stateMachine)
            {
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    this.m_launcherSubsystem.setMotorVelocity(2700);
                    m_stateMachine = StateMachine.PREPARE_FOR_BATTLE;

                    break;

                case PREPARE_FOR_BATTLE:
                    this.m_hoodServoSubsystem.pivotHood(m_aimFar);
                    this.m_axeSubsystem.pivotAxe(kAxeDown);
                    this.m_turnTableSubsystem.Turn(500);
                    this.m_sorterServoSubsystem.spinSorter(-1.0);

                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            Move, 0.5, 0))
                    {
                        holdTimer.reset();
                        // Sorter, Launcher, and Turntable on. Axe down

                        telemetry.addLine("Launch Motor On");
                        m_stateMachine = StateMachine.LAUNCH_BALL_1;
                    }
                    break;

                case LAUNCH_BALL_1:
                {
                    //Transfer On
                    this.m_limitSwitchSubsystem.setTransferPower(-0.35);
                    boolean currentState = m_limitSwitchSubsystem.isPressed();


                    lastState = currentState;

//                        m_limitSwitchSubsystem.setPower(0.0);//To do 1 Test with this as 0.0
                    telemetry.addLine("Transfer On");
                    m_StateTime.reset();
                    resetRuntime();
                    m_stateMachine = StateMachine.WAIT_1;
                }
                break;

                case WAIT_1:
                    // Check if the desired time (e.g., 2.0 seconds) has passed
                    if (m_StateTime.time() > 1.0)
                    {
                        boolean m_currentState = m_limitSwitchSubsystem.isPressed();

                        if (m_currentState && !lastState) {
                            // Time is up, transition to the next state
                            this.m_limitSwitchSubsystem.setTransferPower(0);
                            telemetry.addLine("Wait 1 Done");
                            resetRuntime();
                            m_stateMachine = StateMachine.LAUNCH_BALL_2;
                        }
                    }
                    break;

                case LAUNCH_BALL_2:

                    if (getRuntime() > 1.5)
                    {
                        this.m_limitSwitchSubsystem.setTransferPower(-0.30);
                        boolean currentState = m_limitSwitchSubsystem.isPressed();

                        if (currentState && !lastState)
                    {
                        System.out.println("Hello?");
                        m_limitSwitchSubsystem.incrementHits();
                        m_limitSwitchSubsystem.setTransferPower(0.0);//To do 1 Test with this as 0.0

                    }
                    lastState = currentState;

//                        m_limitSwitchSubsystem.setPower(0.0);//To do 1 Test with this as 0.0
                    telemetry.addLine("Transfer On");
                    m_StateTime.reset();
                    m_stateMachine = StateMachine.WAIT_2;
                }

                break;

                case WAIT_2:
                    // Check if the desired time (e.g., 2.0 seconds) has passed
                    if (m_StateTime.time() > 1.0)
                    {
                        boolean m_currentState = m_limitSwitchSubsystem.isPressed();

                        if (m_currentState && !lastState)
                        {
                            // Time is up, transition to the next state
                            this.m_limitSwitchSubsystem.setTransferPower(0);
                            telemetry.addLine("Wait 1 Done");
                            resetRuntime();
                            m_stateMachine = StateMachine.LAUNCH_BALL_3;
                        }
                    }
                    break;
                case LAUNCH_BALL_3:

                    if (getRuntime() > 1.5)
                    {
                        this.m_limitSwitchSubsystem.setTransferPower(-0.30);
                        boolean currentState = m_limitSwitchSubsystem.isPressed();

                        if (currentState && !lastState)
                        {
                            System.out.println("Hello?");
                            m_limitSwitchSubsystem.incrementHits();
                            m_limitSwitchSubsystem.setTransferPower(0.0);//To do 1 Test with this as 0.0

                        }
                        lastState = currentState;

//                        m_limitSwitchSubsystem.setPower(0.0);//To do 1 Test with this as 0.0
                        telemetry.addLine("Transfer On");
                        m_StateTime.reset();
                        m_stateMachine = StateMachine.WAIT_3;
                    }

                    break;

                case WAIT_3:
                    // Check if the desired time (e.g., 2.0 seconds) has passed
                    if (m_StateTime.time() > 1.0)
                    {
                        boolean m_currentState = m_limitSwitchSubsystem.isPressed();

                        if (m_currentState && !lastState)
                        {
                            // Time is up, transition to the next state
                            this.m_limitSwitchSubsystem.setTransferPower(0);
                            telemetry.addLine("Wait 1 Done");
                            resetRuntime();
                            m_stateMachine = StateMachine.LAUNCH_BALL_4;
                        }
                    }
                    break;

                case LAUNCH_BALL_4:

                    if (getRuntime() > 1.5)//TODO is this the pause time between balls?
                    {
                        this.m_limitSwitchSubsystem.setTransferPower(-0.30);
                        boolean currentState = m_limitSwitchSubsystem.isPressed();

                        if (currentState && !lastState)
                        {
                            System.out.println("Hello?");
                            m_limitSwitchSubsystem.incrementHits();
                            m_limitSwitchSubsystem.setTransferPower(0.0);//To do 1 Test with this as 0.0

                        }
                        lastState = currentState;

//                        m_limitSwitchSubsystem.setPower(0.0);//To do 1 Test with this as 0.0
                        telemetry.addLine("Transfer On");
                        m_StateTime.reset();
                        m_stateMachine = StateMachine.WAIT_4;
                    }

                    break;

                case WAIT_4:
                    // Check if the desired time (e.g., 2.0 seconds) has passed
                    if (m_StateTime.time() > 1.0)//TODO if this is smaller, does it still stop?
                    {
                        boolean m_currentState = m_limitSwitchSubsystem.isPressed();

                        if (m_currentState && !lastState) {
                            // Time is up, transition to the next state
                            this.m_limitSwitchSubsystem.setTransferPower(0);
                            telemetry.addLine("Wait 1 Done");
                            resetRuntime();
                            m_stateMachine = StateMachine.START_PICK_UP;
                        }
                    }
                    break;

                case START_PICK_UP:
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            Park, 0.5, 0))
                    holdTimer.reset();
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            StartPickUp, 0.4, .5))
                    {
                        this.m_axeSubsystem.pivotAxe(kAxeUp);
                        this.m_intakeServo.set(-1.0);
                        m_stateMachine = StateMachine.PICK_UP;


                        telemetry.addLine("Done");
                    }
                    break;

                case PICK_UP:
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            Park, 0.5, 0))
                    holdTimer.reset();
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            EndPickUp, 0.2, 0.5))
                    {
                        this.m_intakeServo.set(0.0);
                        m_stateMachine = StateMachine.PARKED;


                        telemetry.addLine("Done");
                    }
                    break;
                case PARKED:
//                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
//                            Park, 0.5, 0))
                    holdTimer.reset();
                    if (nav.driveTo(new Pose2DUnNormalized(DistanceUnit.MM, m_odo.getPosX(DistanceUnit.MM), m_odo.getPosY(DistanceUnit.MM), UnnormalizedAngleUnit.DEGREES, m_odo.getHeading(UnnormalizedAngleUnit.DEGREES)),
                            Move, 0.4, 0.5))
                    {
                        this.m_axeSubsystem.pivotAxe(kAxeUp);
                        this.m_limitSwitchSubsystem.setTransferPower(0.0);
                        this.m_sorterServoSubsystem.spinSorter(0.0);
                        this.m_launcherSubsystem.setMotorVelocity(0);

                        telemetry.addLine("Done");
                    }
                    break;
            }


        //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
        leftFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
        rightFront.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
        leftBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
        rightBack.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

        telemetry.addData("current state:", m_stateMachine);

//            Pose2DUnNormalized pos = m_odo.getUnNormalizedPosition();

//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(UnnormalizedAngleUnit.DEGREES));
//            telemetry.addData("Position", data);

        telemetry.update();

    }
    }
}   // end class

