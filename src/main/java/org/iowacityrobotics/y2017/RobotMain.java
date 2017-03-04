package org.iowacityrobotics.y2017;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.robot.IRobotProgram;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.util.collection.Pair;
import org.iowacityrobotics.roboed.util.logging.LogLevel;
import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.roboed.util.math.Vector2;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;

public class RobotMain implements IRobotProgram {

    private AHRS ahrs;
    private Sink<Vector4> snkDrive;

    @Override
    public void init() {
        Logs.setLevel(LogLevel.DEBUG);

        // Nav board
        ahrs = new AHRS(SerialPort.Port.kMXP);
        ahrs.reset();
        Source<Double> srcJerk = new Differential()
                .bind(Data.source(() -> (double)ahrs.getRawAccelX()));

        // Drive train
        Source<Vector4> srcDrive = SourceSystems.CONTROL.dualJoy(1)
                .map(MapperSystems.DRIVE.dualJoyMecanum())
                .map(Data.mapper(v -> v.x(v.x() * 0.75).y(v.y() * -0.75).z(v.z() * -1)));
        MotorTuple4 talons = MotorTuple4.ofCANTalons(1, 4, 6, 3);
        talons.getFrontLeft().setInverted(true);
        talons.getFrontRight().setInverted(true);
        snkDrive = SinkSystems.DRIVE.mecanum(talons);

        // Climber
        Source<Double> srcClimb = SourceSystems.CONTROL.button(1, 8)
                .map(MapperSystems.CONTROL.buttonValue(0D, -0.75D));
        Sink<Double> snkClimb = SinkSystems.MOTOR.canTalon(2);

        // Shooter
        Source<Boolean> srcShoot = SourceSystems.CONTROL.button(2, 1);
        Sink<Double> snkShoot = SinkSystems.MOTOR.canTalon(8);

        // Agitator
        Source<Boolean> srcEggBtn = SourceSystems.CONTROL.button(2, 4);
        Source<Boolean> srcEgg = srcEggBtn.inter(srcShoot, Data.inter((a, b) -> a || b));
        Sink<Double> snkEgg = SinkSystems.MOTOR.victorSp(5);

        // Pickup
        Source<Double> srcPickup = SourceSystems.CONTROL.button(2, 2)
                .map(MapperSystems.CONTROL.buttonValue(0D, 0.75D));
        Sink<Double> snkPickup = SinkSystems.MOTOR.canTalon(7);

        // Windshield wiper
        Source<Double> srcWs = SourceSystems.CONTROL.button(2, 3)
                .map(MapperSystems.CONTROL.buttonValue(175D, 115D));
        Sink<Double> snkWs = SinkSystems.MOTOR.servo(0);

        // Ultrasonic sensor
        Source<Double> srcUs = SourceSystems.SENSOR.analog(0)
                .map(Data.mapper(v -> v * 3.4528D));

        // Vision data source
        Source<Pair<Vector4, Vector4>> srcVis = Data.cached(new VisionDataProvider(), 50L);

        // Auto routine control
        SendableChooser<Integer> rtCtrl = new SendableChooser<>();
        rtCtrl.addObject("left", -1);
        rtCtrl.addDefault("center", 0);
        rtCtrl.addObject("right", 1);
        SmartDashboard.putData("routine", rtCtrl);

        // Teleop mode
        RobotMode.TELEOP.setOperation(() -> {
            snkDrive.bind(srcDrive);
            snkClimb.bind(srcClimb);
            snkShoot.bind(srcShoot.map(MapperSystems.CONTROL.buttonValue(0D, 0.9D)));
            snkEgg.bind(srcEgg.map(MapperSystems.CONTROL.buttonValue(0D, 0.8D)));
            snkPickup.bind(srcPickup);
            snkWs.bind(srcWs);
            Flow.waitInfinite();
        });

        // Auto mode
        RobotMode.AUTO.setOperation(() -> {
            final Vector2 vec2 = new Vector2();
            final Vector4 vec4 = new Vector4();

            int routine = rtCtrl.getSelected();
            if (routine != 0) {
                drive(vec2.y(0.35D), 1650L);
                ptTurn(0.35, 30 * Math.signum(routine));
            } else {
                drive(vec2.y(0.35D), 500L);
            }

            drive(vec2.y(0.32D), 325L);
            Flow.waitFor(1000L);
            drive(vec2.y(-0.32D), 325L);

            /*Data.pushState();
            Source<Vector4> srcVisionRotate = srcVis.map(Data.mapper(
                    c -> c == null ? Vector4.ZERO : vec4.z(0.35 * Math.signum(c.getB().w() - c.getA().w()))
            ));
            snkDrive.bind(srcVisionRotate);
            Flow.waitUntil(() -> {
                Pair<Vector4, Vector4> c = srcVis.get();
                return c.getA().w() == c.getB().w();
            });
            Data.popState();*/

            // TODO Strafe

            // TODO Place gear

            // TODO Cross line
        });
    }

    private void drive(Vector2 vec, long time) {
        Data.pushState();
        final Vector4 moveVec = new Vector4(vec.x(), vec.y(), 0, 0);
        snkDrive.bind(Data.source(() -> moveVec));
        Flow.waitFor(time);
        Data.popState();
    }

    private void ptTurn(double speed, float deltaAngle) {
        Data.pushState();
        final Vector4 moveVec = new Vector4(0, 0, -speed * Math.signum(deltaAngle), ahrs.getAngle());
        snkDrive.bind(Data.source(() -> moveVec));
        Flow.waitUntil(new NavAngle(ahrs, deltaAngle));
        Data.popState();
    }

}