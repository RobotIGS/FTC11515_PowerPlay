package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.OmniWheel;

public abstract class BaseTeleOp extends OpMode {
    @Override
    public void init() {
        initialize();
    }

    public abstract void initialize();
}
