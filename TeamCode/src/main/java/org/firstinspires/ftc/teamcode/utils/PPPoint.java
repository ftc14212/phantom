package org.firstinspires.ftc.teamcode.utils;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import java.util.List;

@Config
@Configurable
public class PPPoint {

    public static class beizerLine {
        public double startPointX;
        public double startPointY;
        public double endPointX;
        public double endPointY;
        public double startHeading;
        public double endHeading;
        public beizerLine(double endPointX, double endPointY, double startPointX, double startPointY, double startHeading, double endHeading) {
            this.startPointX = startPointX;
            this.startPointY = startPointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
        }
        public beizerLine(double endPointX, double endPointY, double endHeading) {
            this.startPointX = 0;
            this.startPointY = 0;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = 0;
            this.endHeading = endHeading;
        }
        public Pose getStartPoint() {
            return new Pose(startPointX, startPointY);
        }
        public Pose getEndPoint() {
            return new Pose(endPointX, endPointY);
        }
        public double getStartHeading() {
            return startHeading;
        }
        public double getEndHeading() {
            return endHeading;
        }
    }

    public static class pose {
        public double x;
        public double y;
        public double heading;
        public pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
        public pose(double x, double y) {
            this.x = x;
            this.y = y;
            this.heading = 0;
        }
        public Pose getPose() {
            return new Pose(x, y, heading);
        }
        public double getX() {
            return x;
        }
        public double getY() {
            return y;
        }
        public double getHeading() {
            return heading;
        }
    }

    public static class beizerCurve {
        public double startPointX;
        public double startPointY;
        public double middlePointX;
        public double middlePointY;
        public List<Double> middlePointXL;
        public List<Double> middlePointYL;
        public List<PPMP> middlePointL;
        public double endPointX;
        public double endPointY;
        public double startHeading;
        public double endHeading;
        @Deprecated
        public beizerCurve(double startPointX, double startPointY, double middlePointX, double middlePointY, double endPointX, double endPointY, double startHeading, double endHeading) {
            this.startPointX = startPointX;
            this.startPointY = startPointY;
            this.middlePointX = middlePointX;
            this.middlePointY = middlePointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
        }
        @Deprecated
        public beizerCurve(double startPointX, double startPointY, List<Double> middlePointX, List<Double> middlePointY, double endPointX, double endPointY, double startHeading, double endHeading) {
            this.startPointX = startPointX;
            this.startPointY = startPointY;
            this.middlePointXL = middlePointX;
            this.middlePointYL = middlePointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
        }
        @Deprecated
        public beizerCurve(double middlePointX, double middlePointY, double endPointX, double endPointY, double endHeading) {
            this.startPointX = 0;
            this.startPointY = 0;
            this.middlePointX = middlePointX;
            this.middlePointY = middlePointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = 0;
            this.endHeading = endHeading;
        }
        @Deprecated
        public beizerCurve(List<Double> middlePointX, List<Double> middlePointY, double endPointX, double endPointY, double endHeading) {
            this.startPointX = 0;
            this.startPointY = 0;
            this.middlePointXL = middlePointX;
            this.middlePointYL = middlePointY;
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = 0;
            this.endHeading = endHeading;
        }
        public beizerCurve(double startPointX, double startPointY, double endPointX, double endPointY, double startHeading, double endHeading, PPMP... middlePoints) {
            this.startPointX = startPointX;
            this.startPointY = startPointY;
            this.middlePointL = List.of(middlePoints);
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
        }
        public beizerCurve(double endPointX, double endPointY, double endHeading, PPMP... middlePoints) {
            this.startPointX = 0;
            this.startPointY = 0;
            this.middlePointL = List.of(middlePoints);
            this.endPointX = endPointX;
            this.endPointY = endPointY;
            this.startHeading = 0;
            this.endHeading = endHeading;
        }
        public Pose getStartPoint() {
            return new Pose(startPointX, startPointY);
        }
        public Pose getEndPoint() {
            return new Pose(endPointX, endPointY);
        }
        public Pose getMiddlePoint() {
            return new Pose(middlePointL.get(0).X, middlePointL.get(0).Y);
        }
        public Pose getMiddlePoint(int index) {
            return new Pose(middlePointL.get(index).X, middlePointL.get(index).Y);
        }
        @Deprecated
        public Pose getMiddlePointOLD() {
            return new Pose(middlePointX, middlePointY);
        }
        @Deprecated
        public Pose getMiddlePointOLD(int index) {
            return new Pose(middlePointXL.get(index), middlePointYL.get(index));
        }
        public double getStartHeading() {
            return startHeading;
        }
        public double getEndHeading() {
            return endHeading;
        }
    }
}