package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class WebcamHardwareMap {
    public WebcamName webcam;

    public WebcamHardwareMap(HardwareMap hwMap){
        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }
}
