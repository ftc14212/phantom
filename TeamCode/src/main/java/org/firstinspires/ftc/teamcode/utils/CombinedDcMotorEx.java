package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CombinedDcMotorEx {
    private final List<DcMotorEx> motors = new ArrayList<>();
    public CombinedDcMotorEx(DcMotorEx... motor) {
        this.motors.addAll(Arrays.asList(motor));
    }
    public int getPortNumber(int index) {
        return motors.get(index).getPortNumber();
    }
    public DcMotorController getController(int index) {
        return motors.get(index).getController();
    }
    public void setDirection(DcMotor.Direction direction, int index) {
        motors.get(index).setDirection(direction);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
    public DcMotor.Direction getDirection(int index) {
        return motors.get(index).getDirection();
    }
    public void setPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }
    public void setMode(DcMotor.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }
    public void setTargetPositionTolerance(int tolerance) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPositionTolerance(tolerance);
        }
    }
    public void setVelocity(double velocity) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(velocity);
        }
    }
    public void setMotorType(MotorConfigurationType motorType) {
        for (DcMotorEx motor : motors) {
            motor.setMotorType(motorType);
        }
    }
    public void setCurrentAlert(double current, CurrentUnit unit) {
        for (DcMotorEx motor : motors) {
            motor.setCurrentAlert(current, unit);
        }
    }
    public void setVelocity(double velocity, AngleUnit unit) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(velocity, unit);
        }
    }
    public void setTargetPosition(int position) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(position);
        }
    }
    public void setPIDFCoefficients(DcMotorEx.RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }
    public double getPower() {
        return motors.get(0).getPower();
    }
    public DcMotor.RunMode getMode() {
        return motors.get(0).getMode();
    }
    public int getTargetPosition() {
        return motors.get(0).getTargetPosition();
    }
    public boolean getPowerFloat() {
        return motors.get(0).getPowerFloat();
    }
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motors.get(0).getZeroPowerBehavior();
    }
    public boolean isOverCurrent() {
        return motors.get(0).isOverCurrent();
    }
    public boolean isOverCurrent(int index) {
        return motors.get(index).isOverCurrent();
    }
    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode mode) {
        return motors.get(0).getPIDFCoefficients(mode);
    }
    public double getVelocity() {
        return motors.get(0).getVelocity();
    }
    public double getPower(int index) {
        return motors.get(index).getPower();
    }
    public double getVelocity(int index) {
        return motors.get(index).getVelocity();
    }
    public double getCurrent(CurrentUnit unit) {
        return motors.get(0).getCurrent(unit);
    }
    public double getCurrent(int index, CurrentUnit unit) {
        return motors.get(index).getCurrent(unit);
    }
    public double getCurrentAlert(CurrentUnit unit) {
        return motors.get(0).getCurrentAlert(unit);
    }
    public double getCurrentAlert(int index, CurrentUnit unit) {
        return motors.get(index).getCurrentAlert(unit);
    }
    public int getTargetPositionTolerance() {
        return motors.get(0).getTargetPositionTolerance();
    }
    public MotorConfigurationType getMotorType() {
        return motors.get(0).getMotorType();
    }
    public HardwareDevice.Manufacturer getManufacturer(int index) {
        return motors.get(index).getManufacturer();
    }
    public String getDeviceName(int index) {
        return motors.get(index).getDeviceName();
    }
    public String getConnectionInfo(int index) {
        return motors.get(index).getConnectionInfo();
    }
    public int getVersion(int index) {
        return motors.get(index).getVersion();
    }
    public boolean isBusy() {
        return motors.get(0).isBusy();
    }
    public void close(int index) {
        motors.get(index).close();
    }
    public int getCurrentPosition(int index) {
        return motors.get(index).getCurrentPosition();
    }
    public void setMotorEnable(int index) {
        motors.get(index).setMotorEnable();
    }
    public void setMotorDisable(int index) {
        motors.get(index).setMotorDisable();
    }
    public boolean isMotorEnabled(int index) {
        return motors.get(index).isMotorEnabled();
    }
    public int getMotorsSize() {
        return motors.size();
    }
}