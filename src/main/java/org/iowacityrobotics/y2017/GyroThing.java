package org.iowacityrobotics.y2017;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import org.iowacityrobotics.roboed.api.data.Data;
import org.iowacityrobotics.roboed.api.data.IDataSource;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystemType;
import org.iowacityrobotics.roboed.api.subsystem.provider.IGenericSubsystemProvider;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSourceSubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.FRCSubsystemType;

/**
 * @author Evan Geng and/or Bruno (Mars) Rengel
 */
public class GyroThing extends FRCSourceSubsystem<Double> {

    public static final ISubsystemType<Void, Double, IGenericSubsystemProvider<Void, Double, Void>> TYPE = new FRCSubsystemType<>();

    private final IDataSource<Double> dataSrc;

    public GyroThing() {
        super(TYPE);
        AHRS ahrs = new AHRS(I2C.Port.kMXP);
        ahrs.reset();
        this.dataSrc = Data.provider(ahrs::getAngle);
    }

    @Override
    public IDataSource<Double> output() {
        return dataSrc;
    }

    public static class Provider implements IGenericSubsystemProvider<Void, Double, Void> {

        @Override
        public ISubsystem<Void, Double> getSubsystem(Void ignored) {
            return new GyroThing();
        }

    }

}
