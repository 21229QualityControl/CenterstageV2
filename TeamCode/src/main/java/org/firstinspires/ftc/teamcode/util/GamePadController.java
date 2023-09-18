package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Class that simplifies controller logic.
 * Offers press, hold, release, and more.
 */
@SuppressWarnings({"unused", "FieldMayBeFinal"})
public class GamePadController {
    private static final double triggerThreshold = 0.8;
    private static final double joystickThreshold = 0.75; // Warning, X box controller rightStick +Y seems to max out at ~0.8, probably broken
    private static final int longDuration = 10;
    
    private boolean active = true;

    private Gamepad gamepad;

    private PressDuration dpad_up = new PressDuration(), dpad_down = new PressDuration(), dpad_left = new PressDuration(), dpad_right = new PressDuration();
    private PressDuration x = new PressDuration(), y = new PressDuration(), a = new PressDuration(), b = new PressDuration();
    private PressDuration left_bumper = new PressDuration(), right_bumper = new PressDuration();
    private PressDuration start = new PressDuration(), back = new PressDuration(), guide = new PressDuration();
    private PressDuration left_trigger_it = new PressDuration(), right_trigger_it = new PressDuration();
    private PressDuration left_stick_button = new PressDuration(), right_stick_button = new PressDuration();
    private PressDuration left_stick_px_it = new PressDuration(), left_stick_nx_it = new PressDuration(), left_stick_py_it = new PressDuration(), left_stick_ny_it = new PressDuration();
    private PressDuration right_stick_px_it = new PressDuration(), right_stick_nx_it = new PressDuration(), right_stick_py_it = new PressDuration(), right_stick_ny_it = new PressDuration();

    public double left_stick_x = 0;
    public double right_stick_x = 0;
    public double left_stick_y = 0;
    public double right_stick_y = 0;

    public double left_trigger = 0;
    public double right_trigger = 0;

    public GamePadController(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * Allows activity from this controller
     * @see #disable()
     * @see #isActive()
     */
    public void enable() {
        active = true;
    }

    /**
     * Disables all activity from this controller and disables all flags
     * @see #enable()
     * @see #isActive()
     */
    public void disable() {
        active = false;
        update(); // update twice so everything turns to 0, skipping buttonRelease flag
        update();
    }

    /**
     * Gets whether the controller is enabled
     * @see #enable()
     * @see #disable()
     */
    public boolean isActive() {
        return active;
    }

    /**
     * Gets whether the controller is connected.
     * <p>
     * WARNING: This incorrectly returns false if the controller was
     * plugged in before the program ran and no controller input occurred.
     * @see #isNotPaired()
     * @see #isLostConnection()
     */
    public boolean isConnected() {
        return gamepad.getGamepadId() > 0;
    }

    /**
     * Gets whether the controller was never connected.
     * <p>
     * WARNING: This incorrectly returns true if the controller was
     * plugged in before the program ran and no controller input occurred.
     * @see #isConnected()
     * @see #isLostConnection()
     */
    public boolean isNotPaired() {
        return gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED;
    }

    /**
     * Gets whether the controller was disconnected.
     * <p>
     * This is reliable unlike the others
     * @see #isConnected()
     * @see #isNotPaired()
     */
    public boolean isLostConnection() {
        return gamepad.getGamepadId() == Gamepad.ID_SYNTHETIC;
    }

    /**
     * Counts how many compute cycles something has been pressed
     */
    public static class PressDuration {
        private int n = 0;

        private void update(boolean isPressed) {
            if (isPressed) press();
            else release();
        }
        private void press() {
            n = n<0? 1 : n+1;
        }
        private void release() {
            n = n > 0 ? -n : 0; // on release set to -n, if after release set to 0
        }
    }

    /**
     * Updates the duration each button has been pressed as well as non-boolean values.
     * <p>
     * Must be repeatedly called for this to work.
     */
    public void update() {
        // Buttons
        x.update(gamepad.x && active);
        y.update(gamepad.y && active);
        a.update(gamepad.a && active);
        b.update(gamepad.b && active);
        dpad_up.update(gamepad.dpad_up && active);
        dpad_down.update(gamepad.dpad_down && active);
        dpad_left.update(gamepad.dpad_left && active);
        dpad_right.update(gamepad.dpad_right && active);
        left_bumper.update(gamepad.left_bumper && active);
        right_bumper.update(gamepad.right_bumper && active);
        start.update(gamepad.start && active);
        back.update(gamepad.back && active);
        guide.update(gamepad.guide && active);
        left_stick_button.update(gamepad.left_stick_button && active);
        right_stick_button.update(gamepad.right_stick_button && active);

        // Triggers
        left_trigger_it.update(gamepad.left_trigger > triggerThreshold && active);
        right_trigger_it.update(gamepad.right_trigger > triggerThreshold && active);

        // Left Stick
        left_stick_nx_it.update(gamepad.left_stick_x < -joystickThreshold && active);
        left_stick_px_it.update(gamepad.left_stick_x > joystickThreshold && active);
        left_stick_ny_it.update(gamepad.left_stick_y < -joystickThreshold && active);
        left_stick_py_it.update(gamepad.left_stick_y > joystickThreshold && active);

        // Right Stick
        right_stick_nx_it.update(gamepad.right_stick_x < -joystickThreshold && active);
        right_stick_px_it.update(gamepad.right_stick_x > joystickThreshold && active);
        right_stick_ny_it.update(gamepad.right_stick_y < -joystickThreshold && active);
        right_stick_py_it.update(gamepad.right_stick_y > joystickThreshold && active);

        // Variables
        if (active) { left_stick_x = gamepad.left_stick_x; } else { left_stick_x = 0; }
        if (active) { left_stick_y = gamepad.left_stick_y; } else { left_stick_y = 0; }
        if (active) { right_stick_x = gamepad.right_stick_x; } else { right_stick_x = 0; }
        if (active) { right_stick_y = gamepad.right_stick_y; } else { right_stick_y = 0; }
        if (active) { left_trigger = gamepad.left_trigger; } else { left_trigger = 0; }
        if (active) { right_trigger = gamepad.right_trigger; } else { right_trigger = 0; }
    }

    // isDown (continuously when pressed)
    public boolean dpadUp() { return 0 < dpad_up.n; }
    public boolean dpadDown() { return 0 < dpad_down.n; }
    public boolean dpadLeft() { return 0 < dpad_left.n; }
    public boolean dpadRight() { return 0 < dpad_right.n; }
    public boolean x() { return 0 < x.n; }
    public boolean y() { return 0 < y.n; }
    public boolean a() { return 0 < a.n && !start(); }
    public boolean b() { return 0 < b.n && !start(); }
    public boolean leftBumper() { return 0 < left_bumper.n; }
    public boolean rightBumper() { return 0 < right_bumper.n; }
    public boolean leftTrigger() { return 0 < left_trigger_it.n;}
    public boolean rightTrigger() { return 0 < right_trigger_it.n;}
    public boolean leftStickButton() { return 0 < left_stick_button.n; }
    public boolean rightStickButton() { return 0 < right_stick_button.n; }
    public boolean leftStickRight() { return 0 < left_stick_px_it.n; }
    public boolean leftStickUp() { return 0 < left_stick_ny_it.n; }
    public boolean leftStickLeft() { return 0 < left_stick_nx_it.n; }
    public boolean leftStickDown() { return 0 < left_stick_py_it.n; }
    public boolean rightStickRight() { return 0 < right_stick_px_it.n; }
    public boolean rightStickUp() { return 0 < right_stick_ny_it.n; }
    public boolean rightStickLeft() { return 0 < right_stick_nx_it.n; }
    public boolean rightStickDown() { return 0 < right_stick_py_it.n; }
    public boolean start() { return 0 < start.n;}
    public boolean back() { return 0 < back.n;}
    public boolean guide() { return 0 < guide.n;}

    // isPressed (only once when pressed)
    public boolean dpadUpOnce() { return 1 == dpad_up.n; }
    public boolean dpadDownOnce() { return 1 == dpad_down.n; }
    public boolean dpadLeftOnce() { return 1 == dpad_left.n; }
    public boolean dpadRightOnce() { return 1 == dpad_right.n; }
    public boolean xOnce() { return 1 == x.n; }
    public boolean yOnce() { return 1 == y.n; }
    public boolean aOnce() { return 1 == a.n && !start(); }
    public boolean bOnce() { return 1 == b.n && !start(); }
    public boolean leftBumperOnce() { return 1 == left_bumper.n; }
    public boolean rightBumperOnce() { return 1 == right_bumper.n; }
    public boolean leftTriggerOnce() { return 1 == left_trigger_it.n;}
    public boolean rightTriggerOnce() { return 1 == right_trigger_it.n;}
    public boolean leftStickButtonOnce() { return 1 == left_stick_button.n; }
    public boolean rightStickButtonOnce() { return 1 == right_stick_button.n; }
    public boolean leftStickRightOnce() { return 1 == left_stick_px_it.n; }
    public boolean leftStickUpOnce() { return 1 == left_stick_ny_it.n; }
    public boolean leftStickLeftOnce() { return 1 == left_stick_nx_it.n; }
    public boolean leftStickDownOnce() { return 1 == left_stick_py_it.n; }
    public boolean rightStickRightOnce() { return 1 == right_stick_px_it.n; }
    public boolean rightStickUpOnce() { return 1 == right_stick_ny_it.n; }
    public boolean rightStickLeftOnce() { return 1 == right_stick_nx_it.n; }
    public boolean rightStickDownOnce() { return 1 == right_stick_py_it.n; }
    public boolean startOnce() { return 1 == start.n;}
    public boolean backOnce() { return 1 == back.n;}
    public boolean guideOnce() { return 1 == guide.n;}

    // isLong (continuously after being held)
    public boolean dpadUpLong() { return longDuration <= dpad_up.n; }
    public boolean dpadDownLong() { return longDuration <= dpad_down.n; }
    public boolean dpadLeftLong() { return longDuration <= dpad_left.n; }
    public boolean dpadRightLong() { return longDuration <= dpad_right.n; }
    public boolean xLong() { return longDuration <= x.n; }
    public boolean yLong() { return longDuration <= y.n; }
    public boolean aLong() { return longDuration <= a.n; }
    public boolean bLong() { return longDuration <= b.n; }
    public boolean leftBumperLong() { return longDuration <= left_bumper.n; }
    public boolean rightBumperLong() { return longDuration <= right_bumper.n; }
    public boolean leftTriggerLong() { return longDuration <= left_trigger_it.n;}
    public boolean rightTriggerLong() { return longDuration <= right_trigger_it.n;}
    public boolean leftStickButtonLong() { return longDuration <= left_stick_button.n; }
    public boolean rightStickButtonLong() { return longDuration <= right_stick_button.n; }
    public boolean leftStickRightLong() { return longDuration <= left_stick_px_it.n; }
    public boolean leftStickUpLong() { return longDuration <= left_stick_ny_it.n; }
    public boolean leftStickLeftLong() { return longDuration <= left_stick_nx_it.n; }
    public boolean leftStickDownLong() { return longDuration <= left_stick_py_it.n; }
    public boolean rightStickRightLong() { return longDuration <= right_stick_px_it.n; }
    public boolean rightStickUpLong() { return longDuration <= right_stick_ny_it.n; }
    public boolean rightStickLeftLong() { return longDuration <= right_stick_nx_it.n; }
    public boolean rightStickDownLong() { return longDuration <= right_stick_py_it.n; }
    public boolean startLong() { return longDuration <= start.n;}
    public boolean backLong() { return longDuration <= back.n;}
    public boolean guideLong() { return longDuration <= guide.n;}

    // isLongPressed (only once after being held)
    public boolean dpadUpLongOnce() { return longDuration == dpad_up.n; }
    public boolean dpadDownLongOnce() { return longDuration == dpad_down.n; }
    public boolean dpadLeftLongOnce() { return longDuration == dpad_left.n; }
    public boolean dpadRightLongOnce() { return longDuration == dpad_right.n; }
    public boolean xLongOnce() { return longDuration == x.n; }
    public boolean yLongOnce() { return longDuration == y.n; }
    public boolean aLongOnce() { return longDuration == a.n; }
    public boolean bLongOnce() { return longDuration == b.n; }
    public boolean leftBumperLongOnce() { return longDuration == left_bumper.n; }
    public boolean rightBumperLongOnce() { return longDuration == right_bumper.n; }
    public boolean leftTriggerLongOnce() { return longDuration == left_trigger_it.n;}
    public boolean rightTriggerLongOnce() { return longDuration == right_trigger_it.n;}
    public boolean leftStickButtonLongOnce() { return longDuration == left_stick_button.n; }
    public boolean rightStickButtonLongOnce() { return longDuration == right_stick_button.n; }
    public boolean leftStickRightLongOnce() { return longDuration == left_stick_px_it.n; }
    public boolean leftStickUpLongOnce() { return longDuration == left_stick_ny_it.n; }
    public boolean leftStickLeftLongOnce() { return longDuration == left_stick_nx_it.n; }
    public boolean leftStickDownLongOnce() { return longDuration == left_stick_py_it.n; }
    public boolean rightStickRightLongOnce() { return longDuration == right_stick_px_it.n; }
    public boolean rightStickUpLongOnce() { return longDuration == right_stick_ny_it.n; }
    public boolean rightStickLeftLongOnce() { return longDuration == right_stick_nx_it.n; }
    public boolean rightStickDownLongOnce() { return longDuration == right_stick_py_it.n; }
    public boolean startLongOnce() { return longDuration == start.n;}
    public boolean backLongOnce() { return longDuration == back.n;}
    public boolean guideLongOnce() { return longDuration == guide.n;}

    // isPastDuration (continuously after being held for given iterations)
    public boolean dpadUpPastDuration(double iterations) { return iterations <= dpad_up.n; }
    public boolean dpadDownPastDuration(double iterations) { return iterations <= dpad_down.n; }
    public boolean dpadLeftPastDuration(double iterations) { return iterations <= dpad_left.n; }
    public boolean dpadRightPastDuration(double iterations) { return iterations <= dpad_right.n; }
    public boolean xPastDuration(double iterations) { return iterations <= x.n; }
    public boolean yPastDuration(double iterations) { return iterations <= y.n; }
    public boolean aPastDuration(double iterations) { return iterations <= a.n; }
    public boolean bPastDuration(double iterations) { return iterations <= b.n; }
    public boolean leftBumperPastDuration(double iterations) { return iterations <= left_bumper.n; }
    public boolean rightBumperPastDuration(double iterations) { return iterations <= right_bumper.n; }
    public boolean leftTriggerPastDuration(double iterations) { return iterations <= left_trigger_it.n;}
    public boolean rightTriggerPastDuration(double iterations) { return iterations <= right_trigger_it.n;}
    public boolean leftStickButtonPastDuration(double iterations) { return iterations <= left_stick_button.n; }
    public boolean rightStickButtonPastDuration(double iterations) { return iterations <= right_stick_button.n; }
    public boolean leftStickRightPastDuration(double iterations) { return iterations <= left_stick_px_it.n; }
    public boolean leftStickUpPastDuration(double iterations) { return iterations <= left_stick_ny_it.n; }
    public boolean leftStickLeftPastDuration(double iterations) { return iterations <= left_stick_nx_it.n; }
    public boolean leftStickDownPastDuration(double iterations) { return iterations <= left_stick_py_it.n; }
    public boolean rightStickRightPastDuration(double iterations) { return iterations <= right_stick_px_it.n; }
    public boolean rightStickUpPastDuration(double iterations) { return iterations <= right_stick_ny_it.n; }
    public boolean rightStickLeftPastDuration(double iterations) { return iterations <= right_stick_nx_it.n; }
    public boolean rightStickDownPastDuration(double iterations) { return iterations <= right_stick_py_it.n; }
    public boolean startPastDuration(double iterations) { return iterations <= start.n;}
    public boolean backPastDuration(double iterations) { return iterations <= back.n;}
    public boolean guidePastDuration(double iterations) { return iterations <= guide.n;}

    // isAtDuration (only once after being held for given iterations)
    public boolean dpadUpAtDuration(double iterations) { return iterations == dpad_up.n; }
    public boolean dpadDownAtDuration(double iterations) { return iterations == dpad_down.n; }
    public boolean dpadLeftAtDuration(double iterations) { return iterations == dpad_left.n; }
    public boolean dpadRightAtDuration(double iterations) { return iterations == dpad_right.n; }
    public boolean xAtDuration(double iterations) { return iterations == x.n; }
    public boolean yAtDuration(double iterations) { return iterations == y.n; }
    public boolean aAtDuration(double iterations) { return iterations == a.n; }
    public boolean bAtDuration(double iterations) { return iterations == b.n; }
    public boolean leftBumperAtDuration(double iterations) { return iterations == left_bumper.n; }
    public boolean rightBumperAtDuration(double iterations) { return iterations == right_bumper.n; }
    public boolean leftTriggerAtDuration(double iterations) { return iterations == left_trigger_it.n;}
    public boolean rightTriggerAtDuration(double iterations) { return iterations == right_trigger_it.n;}
    public boolean leftStickButtonAtDuration(double iterations) { return iterations == left_stick_button.n; }
    public boolean rightStickButtonAtDuration(double iterations) { return iterations == right_stick_button.n; }
    public boolean leftStickRightAtDuration(double iterations) { return iterations == left_stick_px_it.n; }
    public boolean leftStickUpAtDuration(double iterations) { return iterations == left_stick_ny_it.n; }
    public boolean leftStickLeftAtDuration(double iterations) { return iterations == left_stick_nx_it.n; }
    public boolean leftStickDownAtDuration(double iterations) { return iterations == left_stick_py_it.n; }
    public boolean rightStickRightAtDuration(double iterations) { return iterations == right_stick_px_it.n; }
    public boolean rightStickUpAtDuration(double iterations) { return iterations == right_stick_ny_it.n; }
    public boolean rightStickLeftAtDuration(double iterations) { return iterations == right_stick_nx_it.n; }
    public boolean rightStickDownAtDuration(double iterations) { return iterations == right_stick_py_it.n; }
    public boolean startAtDuration(double iterations) { return iterations == start.n;}
    public boolean backAtDuration(double iterations) { return iterations == back.n;}
    public boolean guideAtDuration(double iterations) { return iterations == guide.n;}

    // isReleased (only once for any release)
    public boolean dpadUpReleased() { return 0 > dpad_up.n; }
    public boolean dpadDownReleased() { return 0 > dpad_down.n; }
    public boolean dpadLeftReleased() { return 0 > dpad_left.n; }
    public boolean dpadRightReleased() { return 0 > dpad_right.n; }
    public boolean xReleased() { return 0 > x.n; }
    public boolean yReleased() { return 0 > y.n; }
    public boolean aReleased() { return 0 > a.n && !start(); }
    public boolean bReleased() { return 0 > b.n && !start(); }
    public boolean leftBumperReleased() { return 0 > left_bumper.n; }
    public boolean rightBumperReleased() { return 0 > right_bumper.n; }
    public boolean leftTriggerReleased() { return 0 > left_trigger_it.n;}
    public boolean rightTriggerReleased() { return 0 > right_trigger_it.n;}
    public boolean leftStickButtonReleased() { return 0 > left_stick_button.n; }
    public boolean rightStickButtonReleased() { return 0 > right_stick_button.n; }
    public boolean leftStickRightReleased() { return 0 > left_stick_px_it.n; }
    public boolean leftStickUpReleased() { return 0 > left_stick_ny_it.n; }
    public boolean leftStickLeftReleased() { return 0 > left_stick_nx_it.n; }
    public boolean leftStickDownReleased() { return 0 > left_stick_py_it.n; }
    public boolean rightStickRightReleased() { return 0 > right_stick_px_it.n; }
    public boolean rightStickUpReleased() { return 0 > right_stick_ny_it.n; }
    public boolean rightStickLeftReleased() { return 0 > right_stick_nx_it.n; }
    public boolean rightStickDownReleased() { return 0 > right_stick_py_it.n; }
    public boolean startReleased() { return 0 > start.n;}
    public boolean backReleased() { return 0 > back.n;}
    public boolean guideReleased() { return 0 > guide.n;}

    // isReleasedShort (only once for a non-long release)
    public boolean dpadUpReleasedShort() { return -longDuration < dpad_up.n && dpad_up.n < 0; }
    public boolean dpadDownReleasedShort() { return -longDuration < dpad_down.n && dpad_down.n < 0; }
    public boolean dpadLeftReleasedShort() { return -longDuration < dpad_left.n && dpad_left.n < 0; }
    public boolean dpadRightReleasedShort() { return -longDuration < dpad_right.n && dpad_right.n < 0; }
    public boolean xReleasedShort() { return -longDuration < x.n && x.n < 0; }
    public boolean yReleasedShort() { return -longDuration < y.n && y.n < 0; }
    public boolean aReleasedShort() { return -longDuration < a.n && a.n < 0 && !start(); }
    public boolean bReleasedShort() { return -longDuration < b.n && b.n < 0 && !start(); }
    public boolean leftBumperReleasedShort() { return -longDuration < left_bumper.n && left_bumper.n < 0; }
    public boolean rightBumperReleasedShort() { return -longDuration < right_bumper.n && right_bumper.n < 0; }
    public boolean leftTriggerReleasedShort() { return -longDuration < left_trigger_it.n && left_trigger_it.n < 0;}
    public boolean rightTriggerReleasedShort() { return -longDuration < right_trigger_it.n && right_trigger_it.n < 0;}
    public boolean leftStickButtonReleasedShort() { return -longDuration < left_stick_button.n && left_stick_button.n < 0; }
    public boolean rightStickButtonReleasedShort() { return -longDuration < right_stick_button.n && right_stick_button.n < 0; }
    public boolean leftStickRightReleasedShort() { return -longDuration < left_stick_px_it.n && left_stick_px_it.n < 0; }
    public boolean leftStickUpReleasedShort() { return -longDuration < left_stick_ny_it.n && left_stick_ny_it.n < 0; }
    public boolean leftStickLeftReleasedShort() { return -longDuration < left_stick_nx_it.n && left_stick_nx_it.n < 0; }
    public boolean leftStickDownReleasedShort() { return -longDuration < left_stick_py_it.n && left_stick_py_it.n < 0; }
    public boolean rightStickRightReleasedShort() { return -longDuration < right_stick_px_it.n && right_stick_px_it.n < 0; }
    public boolean rightStickUpReleasedShort() { return -longDuration < right_stick_ny_it.n && right_stick_ny_it.n < 0; }
    public boolean rightStickLeftReleasedShort() { return -longDuration < right_stick_nx_it.n && right_stick_nx_it.n < 0; }
    public boolean rightStickDownReleasedShort() { return -longDuration < right_stick_py_it.n && right_stick_py_it.n < 0; }
    public boolean startReleasedShort() { return -longDuration < start.n && start.n < 0;}
    public boolean backReleasedShort() { return -longDuration < back.n && back.n < 0;}
    public boolean guideReleasedShort() { return -longDuration < guide.n && guide.n < 0;}

    // isReleasedLong (only once for a long release)
    public boolean dpadUpReleasedLong() { return -longDuration >= dpad_up.n; }
    public boolean dpadDownReleasedLong() { return -longDuration >= dpad_down.n; }
    public boolean dpadLeftReleasedLong() { return -longDuration >= dpad_left.n; }
    public boolean dpadRightReleasedLong() { return -longDuration >= dpad_right.n; }
    public boolean xReleasedLong() { return -longDuration >= x.n; }
    public boolean yReleasedLong() { return -longDuration >= y.n; }
    public boolean aReleasedLong() { return -longDuration >= a.n && !start(); }
    public boolean bReleasedLong() { return -longDuration >= b.n && !start(); }
    public boolean leftBumperReleasedLong() { return -longDuration >= left_bumper.n; }
    public boolean rightBumperReleasedLong() { return -longDuration >= right_bumper.n; }
    public boolean leftTriggerReleasedLong() { return -longDuration >= left_trigger_it.n;}
    public boolean rightTriggerReleasedLong() { return -longDuration >= right_trigger_it.n;}
    public boolean leftStickButtonReleasedLong() { return -longDuration >= left_stick_button.n; }
    public boolean rightStickButtonReleasedLong() { return -longDuration >= right_stick_button.n; }
    public boolean leftStickRightReleasedLong() { return -longDuration >= left_stick_px_it.n; }
    public boolean leftStickUpReleasedLong() { return -longDuration >= left_stick_ny_it.n; }
    public boolean leftStickLeftReleasedLong() { return -longDuration >= left_stick_nx_it.n; }
    public boolean leftStickDownReleasedLong() { return -longDuration >= left_stick_py_it.n; }
    public boolean rightStickRightReleasedLong() { return -longDuration >= right_stick_px_it.n; }
    public boolean rightStickUpReleasedLong() { return -longDuration >= right_stick_ny_it.n; }
    public boolean rightStickLeftReleasedLong() { return -longDuration >= right_stick_nx_it.n; }
    public boolean rightStickDownReleasedLong() { return -longDuration >= right_stick_py_it.n; }
    public boolean startReleasedLong() { return -longDuration >= start.n;}
    public boolean backReleasedLong() { return -longDuration >= back.n;}
    public boolean guideReleasedLong() { return -longDuration >= guide.n;}

    /**
     * Gets whether none of the inputs are triggering.
     */
    public boolean atRest() {
        return !dpadUp() &&
                !dpadDown() &&
                !dpadLeft() &&
                !dpadRight() &&
                !x() &&
                !y() &&
                !a() &&
                !b() &&
                !leftBumper() &&
                !rightBumper() &&
                !leftTrigger() &&
                !rightTrigger() &&
                !leftStickButton() &&
                !rightStickButton() &&
                !start() &&
                !back() &&
                !guide() &&
                Math.abs(left_stick_x) <= 1e-1 &&
                Math.abs(left_stick_y) <= 1e-1 &&
                Math.abs(right_stick_x) <= 1e-1 &&
                Math.abs(right_stick_y) <= 1e-1 &&
                left_trigger <= 1e-1 &&
                right_trigger <= 1e-1;
    }

    /**
     * Reset all iteration counts to zero.
     */
    public void reset() {
        disable();
        enable();
    }


    /* Rumble Support, copied from Gamepad */

    /**
     * Rumble the gamepad's first rumble motor at maximum power for a certain duration.
     * Calling this will displace any currently running rumble effect.
     * @param durationMs milliseconds to rumble for, or -1 for continuous
     */
    public void rumble(int durationMs) {
        gamepad.rumble(durationMs);
    }

    /**
     * Rumble the gamepad at a fixed rumble power for a certain duration
     * Calling this will displace any currently running rumble effect
     * @param rumble1 rumble power for rumble motor 1 (0.0 - 1.0)
     * @param rumble2 rumble power for rumble motor 2 (0.0 - 1.0)
     * @param durationMs milliseconds to rumble for, or -1 for continuous
     */
    public void rumble(double rumble1, double rumble2, int durationMs) {
        gamepad.rumble(rumble1, rumble2, durationMs);
    }

    /**
     * Rumble the gamepad for a certain number of "blips" using predetermined blip timing
     * This will displace any currently running rumble effect.
     * @param count the number of rumble blips to perform
     */
    public void rumbleBlips(int count) {
        gamepad.rumbleBlips(count);
    }

    /**
     * Run a rumble effect built using {@link Gamepad.RumbleEffect.Builder}
     * The rumble effect will be run asynchronously; your OpMode will
     * not halt execution while the effect is running.
     *
     * Calling this will displace any currently running rumble effect
     */
    public void runRumbleEffect(Gamepad.RumbleEffect effect) {
        gamepad.runRumbleEffect(effect);
    }

    /**
     * Cancel the currently running rumble effect, if any
     */
    public void stopRumble() {
        gamepad.stopRumble();
    }

    /**
     * Returns an educated guess about whether there is a rumble action ongoing on this gamepad
     * @return an educated guess about whether there is a rumble action ongoing on this gamepad
     */
    public boolean isRumbling() {
        return gamepad.isRumbling();
    }

    public Gamepad.RumbleEffect.Builder effectBuilder() {
        return new Gamepad.RumbleEffect.Builder();
    }
}