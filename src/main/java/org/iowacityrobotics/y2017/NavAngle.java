package org.iowacityrobotics.y2017;

import com.kauailabs.navx.frc.AHRS;
import org.iowacityrobotics.roboed.util.function.ICondition;
import org.iowacityrobotics.roboed.util.function.IConditionFactory;

/**
 * Condition that measures delta-angle on the z-axis using MXP nav board.
 * @author Evan Geng
 */
public class NavAngle implements IConditionFactory {

    private final AHRS ahrs;
    private final float deltaDegrees;

    public NavAngle(AHRS ahrs, float deltaDegrees) {
        this.ahrs = ahrs;
        this.deltaDegrees = deltaDegrees;
    }

    @Override
    public ICondition create() {
        return new DeltaAngleCondition();
    }

    private class DeltaAngleCondition implements ICondition {

        private final double initAng;

        private DeltaAngleCondition() {
            this.initAng = ahrs.getAngle();
        }

        @Override
        public boolean isMet() {
            return ahrs.getAngle() - initAng >= deltaDegrees;
        }

    }

}
