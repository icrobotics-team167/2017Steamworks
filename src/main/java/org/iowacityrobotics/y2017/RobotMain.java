package org.iowacityrobotics.y2017;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.robot.IRobotProgram;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.util.logging.LogLevel;
import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;
import org.iowacityrobotics.roboed.vision.CameraType;
import org.iowacityrobotics.roboed.vision.VisionServer;
import org.opencv.core.Mat;

import java.util.function.Supplier;

public class RobotMain implements IRobotProgram {

    public static final double WCLOSED = 103D, WOPEN = 45D;

    private AHRS ahrs;
    private Sink<Vector4> snkDrive;
    private NetworkTable tbl;

    @Override
    public void init() {
        Logs.setLevel(LogLevel.DEBUG);

        Logs.info("Initializing camera stream.");
        Supplier<Mat> cam = VisionServer.getCamera(CameraType.USB, 0);
        VisionServer.putImageSource("usb-cam", cam);

        // Drive train
        Source<Boolean> srcRev = SourceSystems.CONTROL.button(0, 12)
                .map(MapperSystems.CONTROL.toggle());
        Source<Double> srcSneak = SourceSystems.CONTROL.axis(0, 2);
        Source<Vector4> srcDrive = SourceSystems.CONTROL.singleJoy(0)
                .map(MapperSystems.DRIVE.singleJoyMecanum())
                .map(Data.mapper(v -> v.multiply(-1)))
                .inter(srcRev, Data.inter((v, r) -> {
                    double mult = r ? -1 : 1;
                    return v.x(v.x() * mult).y(v.y() * mult);
                }))
                .inter(srcSneak, Data.inter((v, t) -> {
                    double mult = 1 - 0.25 * t;
                    return v.x(v.x() * mult).y(v.y() * mult);
                }));
        MotorTuple4 talons = MotorTuple4.ofCANTalons(1, 4, 6, 3);
        talons.getFrontLeft().setInverted(true);
        talons.getRearLeft().setInverted(true);
        snkDrive = SinkSystems.DRIVE.mecanum(talons);

        // Climber
        Source<Double> srcClimb = SourceSystems.CONTROL.button(0, 11)
                .map(MapperSystems.CONTROL.buttonValue(0D, -1D));
        Sink<Double> snkClimb = SinkSystems.MOTOR.canTalon(2);

        // Shooter
        Source<Boolean> srcShoot = SourceSystems.CONTROL.button(0, 5);
        Sink<Double> snkShoot = SinkSystems.MOTOR.canTalon(8);

        // Agitator
        Source<Boolean> srcEggBtn = SourceSystems.CONTROL.button(0, 3);
        Source<Boolean> srcEgg = srcEggBtn.inter(srcShoot, Data.inter((a, b) -> a || b));
        Sink<Double> snkEgg = SinkSystems.MOTOR.victorSp(8);

        // Pickup
        Source<Double> srcPickup = SourceSystems.CONTROL.button(0, 6)
                .map(MapperSystems.CONTROL.buttonValue(0D, 0.75D));
        Sink<Double> snkPickup = SinkSystems.MOTOR.canTalon(7);

        // Windshield wiper
        //Source<Double> asdf = Data.source(() -> SmartDashboard.getNumber("theServoAngle", 0));
        Source<Double> srcWs = SourceSystems.CONTROL.button(0, 4)
                .map(MapperSystems.CONTROL.buttonValue(WCLOSED, WOPEN));
        Sink<Double> snkWs = SinkSystems.MOTOR.servo(0);

        // Teleop mode
        RobotMode.TELEOP.setOperation(() -> {
            snkDrive.bind(srcDrive);
            snkClimb.bind(srcClimb);
            snkShoot.bind(srcShoot.map(MapperSystems.CONTROL.buttonValue(0D, 1D)));
            snkEgg.bind(srcEgg.map(MapperSystems.CONTROL.buttonValue(0D, 0.8D)));
            snkPickup.bind(srcPickup);
            snkWs.bind(srcWs);
            Flow.waitInfinite();
        });
    }

}