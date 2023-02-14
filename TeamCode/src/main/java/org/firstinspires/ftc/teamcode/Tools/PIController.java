package org.firstinspires.ftc.teamcode.Tools;

public class PIController {
    public double integral = 0;

    private double p;
    private double i;

    public double out = 0;

    public PIController(double p, double i) {
        this.p = p;
        this.i = i;
    }

    public void step(double error) {
        integral += i * error;
        out = integral + (p * error);
    }
}
