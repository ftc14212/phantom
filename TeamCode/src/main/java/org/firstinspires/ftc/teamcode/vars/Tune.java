package org.firstinspires.ftc.teamcode.vars;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class Tune {
    public static class PIDF {
        public double P;
        public double I;
        public double D;
        public double F;

        public PIDF(double P, double I, double D, double F) {
            this.P = P;
            this.I = I;
            this.D = D;
            this.F = F;
        }
    }
    public static class PID {
        public double P;
        public double I;
        public double D;

        public PID(double P, double I, double D) {
            this.P = P;
            this.I = I;
            this.D = D;
        }
    }
}
