package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CombinedGamepad {

    public enum MergeMode {
        PRIORITY_FIRST,   // gamepad[0] overrides others
        AVERAGE_STICKS    // sticks averaged, buttons OR'd
    }

    private final List<Gamepad> gamepads = new ArrayList<>();
    private final MergeMode mode;

    public CombinedGamepad(MergeMode mode, Gamepad... gamepads) {
        this.mode = mode;
        this.gamepads.addAll(Arrays.asList(gamepads));
    }

    public CombinedGamepad(Gamepad... gamepads) {
        this(MergeMode.PRIORITY_FIRST, gamepads);
    }

    /* -------------------------
     * STICKS
     * ------------------------- */

    public double leftStickX() {
        if (mode == MergeMode.AVERAGE_STICKS) {
            return avg(g -> g.left_stick_x);
        }
        return firstNonZero(g -> g.left_stick_x);
    }

    public double leftStickY() {
        if (mode == MergeMode.AVERAGE_STICKS) {
            return avg(g -> g.left_stick_y);
        }
        return firstNonZero(g -> g.left_stick_y);
    }

    public double rightStickX() {
        if (mode == MergeMode.AVERAGE_STICKS) {
            return avg(g -> g.right_stick_x);
        }
        return firstNonZero(g -> g.right_stick_x);
    }

    public double rightStickY() {
        if (mode == MergeMode.AVERAGE_STICKS) {
            return avg(g -> g.right_stick_y);
        }
        return firstNonZero(g -> g.right_stick_y);
    }

    /* -------------------------
     * TRIGGERS
     * ------------------------- */

    public double leftTrigger() {
        return max(g -> g.left_trigger);
    }

    public double rightTrigger() {
        return max(g -> g.right_trigger);
    }

    /* -------------------------
     * BUTTONS (OR logic)
     * ------------------------- */

    public boolean a() {
        return any(g -> g.a);
    }

    public boolean b() {
        return any(g -> g.b);
    }

    public boolean x() {
        return any(g -> g.x);
    }

    public boolean y() {
        return any(g -> g.y);
    }

    public boolean leftBumper() {
        return any(g -> g.left_bumper);
    }

    public boolean rightBumper() {
        return any(g -> g.right_bumper);
    }

    public boolean dpadUp() {
        return any(g -> g.dpad_up);
    }

    public boolean dpadDown() {
        return any(g -> g.dpad_down);
    }

    public boolean dpadLeft() {
        return any(g -> g.dpad_left);
    }

    public boolean dpadRight() {
        return any(g -> g.dpad_right);
    }

    public boolean start() {
        return any(g -> g.start);
    }

    public boolean back() {
        return any(g -> g.back);
    }

    public boolean leftStickButton() {
        return any(g -> g.left_stick_button);
    }

    public boolean rightStickButton() {
        return any(g -> g.right_stick_button);
    }

    /* -------------------------
     * HELPERS
     * ------------------------- */

    private boolean any(Condition c) {
        for (Gamepad g : gamepads) {
            if (c.test(g)) return true;
        }
        return false;
    }

    private double avg(Value v) {
        double sum = 0;
        int count = 0;

        for (Gamepad g : gamepads) {
            sum += v.get(g);
            count++;
        }

        return count == 0 ? 0 : sum / count;
    }

    private double max(Value v) {
        double max = 0;
        for (Gamepad g : gamepads) {
            max = Math.max(max, v.get(g));
        }
        return max;
    }

    private double firstNonZero(Value v) {
        for (Gamepad g : gamepads) {
            double val = v.get(g);
            if (Math.abs(val) > 0.05) return val;
        }
        return 0;
    }

    /* -------------------------
     * FUNCTIONAL INTERFACES
     * ------------------------- */

    private interface Condition {
        boolean test(Gamepad g);
    }

    private interface Value {
        double get(Gamepad g);
    }
}