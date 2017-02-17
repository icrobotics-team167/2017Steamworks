package org.iowacityrobotics.y2017;

import edu.wpi.first.wpilibj.VictorSP;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.IGenericSubsystemProvider;
import org.iowacityrobotics.roboed.api.subsystem.provider.ISinglePortSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;
import org.iowacityrobotics.roboed.impl.subsystem.FRCTerminalSubsystem;

/**
 * Climbs stuff when the input is true.
 * @author Evan Geng
 */
public class SpinningEggMachine extends FRCTerminalSubsystem<Boolean> {

    public static final ISubsystemType<Boolean, Void, IGenericSubsystemProvider<Boolean, Void, VictorSP>> TYPE = new FRCSubsystemType<>();
    private static final double SPEED = -0.75;

    private final VictorSP motor;

    protected SpinningEggMachine(VictorSP motor) {
        super(TYPE);
        this.motor = motor;
    }

    @Override
    protected void processData(Boolean data) {
        motor.set(data ? SPEED : 0D);
    }

    public static class Provider implements IGenericSubsystemProvider<Boolean, Void, VictorSP> {

        @Override
        public ISubsystem<Boolean, Void> getSubsystem(VictorSP victor) {
            return new SpinningEggMachine(victor);
        }

    }

}
