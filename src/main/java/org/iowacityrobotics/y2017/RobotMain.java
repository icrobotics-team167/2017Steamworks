package org.iowacityrobotics.y2017;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
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

        // Gyro
        ahrs = new AHRS(SerialPort.Port.kMXP);
        ahrs.reset();

        // Drive train
        Source<Vector4> srcDrive = SourceSystems.CONTROL.dualJoy(1)
                .map(MapperSystems.DRIVE.dualJoyMecanum())
                .map(Data.mapper(v -> v.x(v.x() * 0.75).y(v.y() * -0.75).z(v.z() * -1)));
        MotorTuple4 talons = MotorTuple4.ofCANTalons(1, 4, 6, 3);
        talons.getFrontLeft().setInverted(true);
        talons.getFrontRight().setInverted(true);
        snkDrive = SinkSystems.DRIVE.mecanum(talons);

        // Climber
        Source<Double> srcClimb = SourceSystems.CONTROL.button(2, 8)
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

        // Smart Dashboard stuff
        Sink<Double> snkDb = SinkSystems.DASH.number("ultrasonic");

        // Vision data source
        Source<Pair<Vector4, Vector4>> srcVis = new VisionDataProvider();

        // Teleop mode
        RobotMode.TELEOP.setOperation(() -> {
            snkDrive.bind(srcDrive);
            snkClimb.bind(srcClimb);
            snkShoot.bind(srcShoot.map(MapperSystems.CONTROL.buttonValue(0D, 0.9D)));
            snkEgg.bind(srcEgg.map(MapperSystems.CONTROL.buttonValue(0D, 0.8D)));
            snkPickup.bind(srcPickup);
            snkWs.bind(srcWs);
            snkDb.bind(srcUs);
            Flow.waitInfinite();
        });

        // Auto mode
        RobotMode.AUTO.setOperation(() -> {
            /*drive(new Vector2(0, 0.3), 1000L);
            ptTurn(0.36, 90F);
            drive(new Vector2(0, 0.3), 1000L);
            ptTurn(0.36, 90F);
            drive(new Vector2(0, 0.3), 1000L);
            ptTurn(0.36, 90F);
            drive(new Vector2(0, 0.3), 1000L);*/
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
