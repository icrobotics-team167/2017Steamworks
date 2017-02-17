package org.iowacityrobotics.y2017;

import edu.wpi.first.wpilibj.Servo;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.ISinglePortSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;
import org.iowacityrobotics.roboed.impl.subsystem.FRCTerminalSubsystem;

/**
 * Created by Ethan Chen
 */
//Supposed to turn a servo 55 degrees

public class WindshieldSubsystem extends FRCTerminalSubsystem<Boolean> {

    public static final ISubsystemType<Boolean, Void, ISinglePortSubsystemProvider<Boolean, Void>> TYPE = new FRCSubsystemType<>();

    private Servo servo;
    private boolean cache;
    private boolean wasPressed = false;
    private double angle1;
    private double angle2;

    protected WindshieldSubsystem(int p) {
        super(TYPE);
        //Initializes all dat shit
        angle1 = 175;
        angle2 = 115;
        servo = new Servo(p);
    }

    protected void turn() {
        //If cache is true, sets to angle2; otherwise sets to angle1
        servo.setAngle(cache ? angle2 : angle1);
    }

    @Override
    protected void processData(Boolean isPressed) {
        //If button was not pressed in previous frame and now is pressed,
        //Cache is switched, wasPressed (pressed in previous frame) is now true
        if (isPressed) {
            if (!wasPressed) {
                cache = !cache;
                wasPressed = true;
            }
        } else {
            wasPressed = false;
        }
        this.turn();
    }

    public static class Provider implements ISinglePortSubsystemProvider<Boolean, Void> {

        @Override
        public ISubsystem<Boolean, Void> getSubsystem(int port) {
            return new WindshieldSubsystem(port);
        }
    }
}