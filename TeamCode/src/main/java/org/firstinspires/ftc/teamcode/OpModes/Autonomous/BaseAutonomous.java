package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.BarcodeEnum;
import org.firstinspires.ftc.teamcode.Tools.ColorEnum;
import org.firstinspires.ftc.teamcode.Tools.ColorTools;
import org.firstinspires.ftc.teamcode.Tools.ControlledDrive;
import org.firstinspires.ftc.teamcode.Tools.OmniWheel;
import org.firstinspires.ftc.teamcode.Tools.PositionEnum;

import java.util.Date;

public abstract class BaseAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        run();
    }

    void initialize() {
    }

    public abstract BaseHardwareMap initializeHardwareMap();
    
    public abstract void run();
}
