package frc.robot.utils;

import javax.management.RuntimeErrorException;

public record PIDConstantsTempName(double P, double I, double D, double FF, double IZone) { /*AAAAAAAAAAAAAAAAAAAAAAA*/
    public PIDConstantsTempName {
        if(P < -1 || P > 1) throw new RuntimeErrorException(new Error("invalid P value (-1 >= p >= 1)"));
        if(I < -1 || I > 1) throw new RuntimeErrorException(new Error("invalid P value (-1 >= I >= 1)"));
        if(D < -1 || D > 1) throw new RuntimeErrorException(new Error("invalid P value (-1 >= p >= 1)"));
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public double getFF() {
        return FF;
    }

    public double getIZone() {
        return IZone;
    }
}