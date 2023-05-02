package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.WebcamHardwareMap;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.BaseTeleOp;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TestOpenCV;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;
import org.firstinspires.ftc.teamcode.Tools.JUNCTION_DRIVE_STATE;
import org.firstinspires.ftc.teamcode.Tools.JunctionDrive;
import org.firstinspires.ftc.teamcode.Tools.JUNCTION_DRIVE;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Date;

@Autonomous
public class TestJunctionDrive extends BaseTeleOp {
    private BaseHardwareMap robot;
    private GyroHardwareMap gyro;
    public FieldNavigation navi;
    public WebcamHardwareMap webcamHardwareMap;
    public OpenCvCamera phoneCam;
    protected JunctionDrive junctiondrive;

    boolean RUN = true;
    protected JUNCTION_DRIVE action;

    double start_rotation;
    double last_rotation;
    String outputBuffer = "";

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, gyro,0.0,0.0,0.0,0.0,0.0);
        OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        junctiondrive = new JunctionDrive(phoneCam);

        start_rotation = navi.rotation_y;
        last_rotation = start_rotation;
    }

    @Override
    public void loop() {
        long startT = (new Date()).getTime();
        telemetry.addData("", "%3.2f %3.2f %3.2f (L) [%s]", junctiondrive.pipeline.returnScalars[0].val[0], junctiondrive.pipeline.returnScalars[0].val[1], junctiondrive.pipeline.returnScalars[0].val[2], junctiondrive.isStick(0)?"True":"False");
        telemetry.addData("", "%3.2f %3.2f %3.2f (M) [%s]", junctiondrive.pipeline.returnScalars[1].val[0], junctiondrive.pipeline.returnScalars[1].val[1], junctiondrive.pipeline.returnScalars[1].val[2], junctiondrive.isStick(1)?"True":"False");
        telemetry.addData("", "%3.2f %3.2f %3.2f (R) [%s]", junctiondrive.pipeline.returnScalars[2].val[0], junctiondrive.pipeline.returnScalars[2].val[1], junctiondrive.pipeline.returnScalars[2].val[2], junctiondrive.isStick(2)?"True":"False");
        telemetry.addData("rotation", junctiondrive.getRotation());

        telemetry.addLine("\nACTIONS : \n");

        if (RUN) {
            action = junctiondrive.step();
            switch (action) {
                case DRIVE_FORW:
                    //telemetry.addLine("Foward");
                    outputBuffer += "Forward\n";
                    navi.drive_setSpeed(-1,0,0,0.28);
                    break;
                case END2:
                    //telemetry.addLine("Parking");
                    outputBuffer += "Parking\n";
                    navi.drive_setSpeed(0,0,0,0);
                    RUN = false;
                    break;
                case ROT_LEFT:
                    //telemetry.addLine("Rotate left");
                    outputBuffer += "ROT Left\n";
                    navi.drive_setSpeed(0, 0, -1, 0.28);
                    break;
                case ROT_RIGHT:
                    //telemetry.addLine("Rotate Right");
                    outputBuffer += "ROT Right\n";
                    navi.drive_setSpeed(0, 0, 1, 0.28);
                    break;
                case SCORE:
                    //telemetry.addLine("Place cone on junction");
                    outputBuffer += "Score\n";
                    navi.drive_setSpeed(0, 0, 0, 0);
                    RUN = false;
                    break;
                case SKIP:
                    //telemetry.addLine("Nothing");
                    outputBuffer += "Nothing\n";
                    break;

            }
        } else {
            switch (action) {
                case END2:
                    telemetry.addLine("end : END2");break;
                case SCORE:
                    telemetry.addLine("end : SCORE");break;
                default:
                    telemetry.addLine("end : ?????");break;
            }
            telemetry.addLine("DONE");
        }
        telemetry.addLine(outputBuffer);
        telemetry.update();

        // update navi
        navi.step();

        last_rotation = navi.rotation_y;
        junctiondrive.setRotation(start_rotation - navi.rotation_y);
    }
}