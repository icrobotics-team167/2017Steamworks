package org.iowacityrobotics.y2017;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.iowacityrobotics.roboed.api.data.Data;
import org.iowacityrobotics.roboed.api.data.IDataSource;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.ISinglePortSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSourceSubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.SingleJoySubsystem;

/**
 * @author Probably not Ethan Chen, not Charlie Zheng, and definitely not Hang Sun
 */

public class ButtonSubsystem extends FRCSourceSubsystem<Boolean> {

    public static final ISubsystemType<Void, Boolean, ISinglePortSubsystemProvider<Void, Boolean>> TYPE = new FRCSubsystemType<>();

    private final IDataSource<Boolean> upstream;

    protected ButtonSubsystem() {
        super(TYPE);

        DriverStation ds = DriverStation.getInstance();
        upstream = Data.provider(() -> ds.getStickButton(1, (byte) 1));
    }

    @Override
    public IDataSource<Boolean> output() {
        return upstream;
    }

    public static class Provider implements ISinglePortSubsystemProvider<Void, Boolean> {

        @Override
        public ISubsystem<Void, Boolean> getSubsystem(int port) {
            return new ButtonSubsystem();
        }

    }

}
