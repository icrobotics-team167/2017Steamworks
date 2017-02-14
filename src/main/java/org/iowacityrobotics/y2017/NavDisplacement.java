package org.iowacityrobotics.y2017;

import com.kauailabs.navx.frc.AHRS;
import org.iowacityrobotics.roboed.util.function.ICondition;
import org.iowacityrobotics.roboed.util.function.IConditionFactory;
import org.iowacityrobotics.roboed.util.math.Vector2;

import java.util.function.DoubleSupplier;

/**
 * Condition that measures displacement using MXP nav board.
 * @author Evan Geng
 */
public class NavDisplacement implements IConditionFactory {

    private final AHRS ahrs;
    private final double meters;

    public NavDisplacement(AHRS ahrs, double meters) {
        this.ahrs = ahrs;
        this.meters = meters;
    }

    @Override
    public ICondition create() {
        return new DisplacementCondition();
    }

    private class DisplacementCondition implements ICondition {

        private final double initX, initY;

        private DisplacementCondition() {
            this.initX = ahrs.getDisplacementX();
            this.initY = ahrs.getDisplacementY();
        }

        @Override
        public boolean isMet() {
            return Math.hypot(
                    ahrs.getDisplacementX() - initX,
                    ahrs.getDisplacementY() - initY
            ) >= meters;
        }

    }

}
