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
import org.firstinspires.ftc.teamcode.Tools.JunctionDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Date;

@Autonomous
public class TestJunctionDrive extends BaseTeleOp {
    WebcamHardwareMap webcamHardwareMap;
    OpenCvCamera phoneCam;
    protected JunctionDrive junctiondrive;

    @Override
    public void initialize() {
        OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        junctiondrive = new JunctionDrive(phoneCam);
    }

    @Override
    public void loop() {
        telemetry.addData("","%3.2f %3.2f %3.2f (L)", junctiondrive.pipeline.returnScalars[0].val[0],junctiondrive.pipeline.returnScalars[0].val[0],junctiondrive.pipeline.returnScalars[0].val[0]);
        telemetry.addData("","%3.2f %3.2f %3.2f (M)", junctiondrive.pipeline.returnScalars[1].val[0],junctiondrive.pipeline.returnScalars[1].val[0],junctiondrive.pipeline.returnScalars[1].val[0]);
        telemetry.addData("","%3.2f %3.2f %3.2f (R)", junctiondrive.pipeline.returnScalars[2].val[0],junctiondrive.pipeline.returnScalars[2].val[0],junctiondrive.pipeline.returnScalars[2].val[0]);
        telemetry.update();
        long startT = (new Date()).getTime();
        while (startT+1000 > (new Date()).getTime()) {}
    }


}