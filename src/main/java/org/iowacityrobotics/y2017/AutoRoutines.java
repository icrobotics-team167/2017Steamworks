package org.iowacityrobotics.y2017;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.iowacityrobotics.roboed.api.data.Data;
import org.iowacityrobotics.roboed.api.operations.IOpMode;
import org.iowacityrobotics.roboed.api.operations.IOperationsManager;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.MecanumSubsystem;
import org.iowacityrobotics.roboed.util.function.TimeDelayCondition;
import org.iowacityrobotics.roboed.util.math.Vector2;

import java.util.concurrent.TimeUnit;

/**
 * Autonomous routine generation functions.
 * @author Evan Geng
 */
public class AutoRoutines {

    private final IOperationsManager opMan;
    private final ISubsystem<MecanumSubsystem.ControlDataFrame, Void> drive;
    private final AHRS ahrs;

    public AutoRoutines(IOperationsManager opMan, ISubsystem<MecanumSubsystem.ControlDataFrame, Void> drive, AHRS ahrs) {
        this.opMan = opMan;
        this.drive = drive;
        this.ahrs = ahrs;
    }

    public IOpMode driveTime(String id, Vector2 dir, long time) {
        IOpMode mode = opMan.getOpMode(id);
        mode.onInit(() -> {
            ahrs.reset();
            drive.bind(Data.provider(() -> new MecanumSubsystem.ControlDataFrame(dir, 0, ahrs.getAngle())));
        });
        mode.untilCondition(() -> new TimeDelayCondition(time));
        return mode;
    }

    public IOpMode driveAng(String id, double pointTurn, float da) {
        IOpMode mode = opMan.getOpMode(id);
        mode.onInit(() -> {
            ahrs.reset();
            drive.bind(Data.provider(() -> new MecanumSubsystem.ControlDataFrame(Vector2.ZERO, pointTurn, ahrs.getAngle())));
        });
        mode.untilCondition(new NavAngle(ahrs, da));
        return mode;
    }

    public IOpMode drive(String id, Vector2 dir, double meters) {
        IOpMode mode = opMan.getOpMode(id);
        mode.onInit(() -> {
            ahrs.reset();
            drive.bind(Data.provider(() ->
                new MecanumSubsystem.ControlDataFrame(dir, 0, ahrs.getAngle())));
            ahrs.resetDisplacement();
        });
        mode.onDone(() -> {
            SmartDashboard.putNumber("f disp x", ahrs.getDisplacementX());
        });
        //mode.untilCondition(new NavDisplacement(ahrs, meters));
        mode.untilCondition(new TimeDelayCondition(500, TimeUnit.MILLISECONDS));
        return mode;
    }

}
