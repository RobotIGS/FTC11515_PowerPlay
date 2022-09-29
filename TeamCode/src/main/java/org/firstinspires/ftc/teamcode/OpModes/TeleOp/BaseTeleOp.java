package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class BaseTeleOp extends OpMode {
    @Override
    public void init() {
        initialize();
    }

    public abstract void initialize();
}
