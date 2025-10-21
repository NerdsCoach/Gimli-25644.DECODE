//package teamCode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//@TeleOp(name = "TouchTest")
//public class TouchSensor extends LinearOpMode
//{
//    private com.qualcomm.robotcore.hardware.TouchSensor m_touchSensor;//in Robot Container
//    private boolean touchSensorIsPressed = false;// in Robot Container
//    private double touchSensorValue;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        initHardware();
//        while (!isStarted())//Not needed??
//        {
////            getTouchSensor();
//            touchTelemetry();
//        }
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
////            getTouchSensor();
//            touchTelemetry();
//        }
//
//    }
//    public void initTouchSensor()// in Robot Container and Command
//    {
//        m_touchSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.TouchSensor.class, "intakeTouchSensor");
//    }
//
//    public void initHardware()//Currently not in Sting-Ray
//    {
//        initTouchSensor();
//    }
//
////    public void getTouchSensor()//in Intake Wheel Subsystem
////    {
////        touchSensorIsPressed = m_touchSensor.isPressed();
//////        touchSensorValue = m_touchSensor.getValue();
////    }
//
//    public void touchTelemetry()//Concept goes into Intake Wheel Command
//        {
//            telemetry.addData("touchSensorPressed", touchSensorIsPressed);
////            telemetry.addData("touchSensorValue", "%.2f",touchSensorValue);
//            telemetry.update();
//
//
//    }
//}