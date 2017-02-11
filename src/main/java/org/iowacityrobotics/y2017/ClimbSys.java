package org.iowacityrobotics.y2017;

import com.ctre.CANTalon;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.ISinglePortSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;
import org.iowacityrobotics.roboed.impl.subsystem.FRCTerminalSubsystem;

/**
 * Climbs stuff when the input is true.
 * @author Evan Geng
 */
public class ClimbSys extends FRCTerminalSubsystem<Boolean> {

    public static final ISubsystemType<Boolean, Void, ISinglePortSubsystemProvider<Boolean, Void>> TYPE = new FRCSubsystemType<>();
    private static final double SPEED = 0.75;

    private final CANTalon motor;

    protected ClimbSys(CANTalon motor) {
        super(TYPE);
        this.motor = motor;
    }

    @Override
    protected void processData(Boolean data) {
        motor.set(data ? SPEED : 0D);
    }

    public static class Provider implements ISinglePortSubsystemProvider<Boolean, Void> {

        @Override
        public ISubsystem<Boolean, Void> getSubsystem(int port) {
            return new ClimbSys(new CANTalon(port));
        }

    }

}
