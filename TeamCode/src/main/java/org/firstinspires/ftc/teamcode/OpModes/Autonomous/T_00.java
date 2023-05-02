package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Tools.JunctionDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="testing drive junction cv", group="test")
public class T_00 extends FullAutonomous{
    @Override
    public void detectSignal() {}

    @Override
    protected void initialize() {
        super.initialize();
        OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        junctiondrive = new JunctionDrive(phoneCam);
        telemetry.addLine("init: Done");
        telemetry.update();
    }

    @Override
    public void run() {
        driveToJunctionCV();
    }
}
