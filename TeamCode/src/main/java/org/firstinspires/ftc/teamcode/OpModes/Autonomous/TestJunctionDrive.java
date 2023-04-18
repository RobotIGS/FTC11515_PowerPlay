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
        telemetry.addData("", "%3.2f %3.2f %3.2f (L)", junctiondrive.pipeline.returnScalars[0].val[0], junctiondrive.pipeline.returnScalars[0].val[1], junctiondrive.pipeline.returnScalars[0].val[2]);
        telemetry.addData("", "%3.2f %3.2f %3.2f (M)", junctiondrive.pipeline.returnScalars[1].val[0], junctiondrive.pipeline.returnScalars[1].val[1], junctiondrive.pipeline.returnScalars[1].val[2]);
        telemetry.addData("", "%3.2f %3.2f %3.2f (R)", junctiondrive.pipeline.returnScalars[2].val[0], junctiondrive.pipeline.returnScalars[2].val[1], junctiondrive.pipeline.returnScalars[2].val[2]);

        telemetry.addLine("\nACTION : \n");

        if (RUN) {
            action = junctiondrive.step();
            switch (action) {
                case DRIVE_FORW:
                    telemetry.addLine("Foward");
                    navi.drive_setMotors(1,0,0,0.5);
                    break;
                case END2:
                    navi.drive_setMotors(0,0,0,0);
                    telemetry.addLine("Parking");
                    RUN = false;
                    break;
                case ROT_LEFT:
                    telemetry.addLine("Rotate left");
                    navi.drive_setMotors(0, 0, -1, 0.3);
                    break;
                case ROT_RIGHT:
                    telemetry.addLine("Rotate Right");
                    navi.drive_setMotors(0, 0, 1, 0.3);
                    break;
                case SCORE:
                    telemetry.addLine("Place cone on junction");
                    navi.drive_setMotors(0, 0, 0, 0);
                    RUN = false;
                    break;
                case SKIP:
                    telemetry.addLine("Nothing");
                    break;

            }
        } else {
            switch (action) {
                case END2:
                    telemetry.addLine("END2");break;
                case SCORE:
                    telemetry.addLine("SCORE");break;
                default:
                    telemetry.addLine("?????");break;
            }
            telemetry.addLine("DONE");
        }
        telemetry.update();

        // update navi
        if (action == JUNCTION_DRIVE.ROT_LEFT || action == JUNCTION_DRIVE.ROT_RIGHT) {
            navi.stepRotation();
            navi.stepPos();
        } else {
            navi.step();
        }

        last_rotation = navi.rotation_y;
        junctiondrive.setRotation(start_rotation - navi.rotation_y);

        while (startT+1000 > (new Date()).getTime()) {}
    }


}