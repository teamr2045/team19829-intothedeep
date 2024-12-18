package org.firstinspires.ftc.teamcode.controllers;

public final class PIDCoefficients {
    public double kP, kI, kD;
    public PIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
