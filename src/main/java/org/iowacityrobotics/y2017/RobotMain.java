package org.iowacityrobotics.y2017;

import com.ctre.CANTalon;
import org.iowacityrobotics.roboed.api.IRobot;
import org.iowacityrobotics.roboed.api.IRobotProgram;
import org.iowacityrobotics.roboed.api.RobotMode;
import org.iowacityrobotics.roboed.api.data.IDataSource;
import org.iowacityrobotics.roboed.api.operations.IOpMode;
import org.iowacityrobotics.roboed.api.subsystem.ISubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.DualJoySubsystem;
import org.iowacityrobotics.roboed.impl.subsystem.impl.MecanumSubsystem;
import org.iowacityrobotics.roboed.util.collection.Pair;
import org.iowacityrobotics.roboed.util.math.Vector2;
import org.iowacityrobotics.roboed.util.robot.QuadraSpeedController;

import java.util.function.BiFunction;

public class RobotMain implements IRobotProgram {

    private ISubsystem<Void, Pair<Vector2, Vector2>> joy;
    private ISubsystem<Void, Double> gyro;
    private ISubsystem<MecanumSubsystem.ControlDataFrame, Void> driveTrain;
    private ISubsystem<Boolean, Void> climber;
    private ISubsystem<Void, Boolean> climbBtn;

    @Override
    public void init(IRobot robot) {
        System.out.println("INIT");
        // Register custom subsystem type
        //robot.getSystemRegistry().registerProvider(ShooterSubsystem.TYPE, new ShooterSubsystem.Provider());
        robot.getSystemRegistry().registerProvider(GyroThing.TYPE, new GyroThing.Provider());
        robot.getSystemRegistry().registerProvider(ClimbSys.TYPE, new ClimbSys.Provider());

        // Initialize subsystems
        joy = robot.getSystemRegistry().getProvider(DualJoySubsystem.TYPE).getSubsystem(1);
        gyro = robot.getSystemRegistry().getProvider(GyroThing.TYPE).getSubsystem(null);
        QuadraSpeedController talons = new QuadraSpeedController(
                new CANTalon(3),
                new CANTalon(4),
                new CANTalon(2),
                new CANTalon(5)
        );
        talons.getC().setInverted(true);
        talons.getD().setInverted(true);
        driveTrain = robot.getSystemRegistry().getProvider(MecanumSubsystem.TYPE_CUSTOM).getSubsystem(talons);
        climber = robot.getSystemRegistry().getProvider(ClimbSys.TYPE).getSubsystem(5);
        climbBtn = robot.getSystemRegistry().getProvider(ButtonSubsystem.TYPE).getSubsystem(1, 1);

        // Set up standard teleop opmode
        IOpMode stdMode = robot.getOpManager().getOpMode("standard");
        final double threshold = 0.1D;
        stdMode.onInit(() -> {
            IDataSource<Pair<Vector2, Vector2>> joyData = joy.output()
                    .map(v -> Pair.of(v.getA().multiply(0.75D), v.getB().multiply(0.75D)));
            IDataSource<Double> gyroData = gyro.output()
                    .map(d -> d * -1);
            BiFunction<Pair<Vector2, Vector2>, Double, MecanumSubsystem.ControlDataFrame> driveFunc
                    = (j, g) -> new MecanumSubsystem.ControlDataFrame(j.getA(), j.getB().x(), g);
            driveTrain.bind(joyData.interpolate(gyroData, driveFunc));
            climber.bind(climbBtn.output());
        });
        stdMode.whileCondition(() -> true);
        robot.getOpManager().setDefaultOpMode(RobotMode.TELEOP, "standard");
/*
        // Start Autonomous
        //1 forward 7ish? ft
        //2 pivot turn 60 degrees
        //3 vision tracking to find how much to strafe left or right
        //4 lever/ws-wiper
        //5 forward ? ft
        //6 backward
        int scenario = 1;
        //0 = middle; -1 = left; 1 = right
        long d = 0; // distance from l/r

        //1st step - fwd
        IOpMode autoMode = robot.getOpManager().getOpMode("auto_init");
        autoMode.onInit(() -> {
            //initializes subsystems: (Drive Train) bind constant .5
            driveTrain.bind(Data.constant(new MecanumSubsystem.ControlDataFrame(new Vector2(0, 0.75), 0)));
        });
        if (scenario == 0)
            autoMode.untilCondition(() -> new TimeDelayCondition(2, TimeUnit.SECONDS)); //fwd all the way to peg
        else
            autoMode.untilCondition(() -> new TimeDelayCondition(3, TimeUnit.SECONDS)); //fwd enough to turn
        autoMode.setNext("auto_part_2");
        robot.getOpManager().setDefaultOpMode(RobotMode.AUTO, "auto_init");

        //2nd step - turn
        autoMode = robot.getOpManager().getOpMode("auto_part_2");
        autoMode.onInit(() -> {
                driveTrain.bind(Data.constant(new MecanumSubsystem.ControlDataFrame(new Vector2(0, 0), scenario *.75)));
        });
        autoMode.untilCondition(() -> new TimeDelayCondition(2, TimeUnit.SECONDS));//set angle to 30 degrees
        autoMode.setNext("auto_part_3");

        //3rd step - strf w vision
        autoMode = robot.getOpManager().getOpMode("auto_part_3");
        autoMode.onInit(() -> {
            //vision trk sets these
            int x = 1;
            d = 2;
            driveTrain.bind(Data.constant(new MecanumSubsystem.ControlDataFrame(new Vector2(x * .75, 0), 0)));
        });

        autoMode.untilCondition(() -> new TimeDelayCondition(2, TimeUnit.SECONDS)); //strafe for d distance
        autoMode.setNext("auto_part_4");

        //4th step - release
        autoMode = robot.getOpManager().getOpMode("auto_part_4");
        autoMode.onInit(() -> {
            //move wiper/lever
        });
        autoMode.untilCondition(() -> new TimeDelayCondition(1 / 2, TimeUnit.SECONDS));
        autoMode.setNext("auto_part_5");

        //5th step - fwd
        autoMode = robot.getOpManager().getOpMode("auto_part_5");
        autoMode.onInit(() -> {
            driveTrain.bind(Data.constant(new MecanumSubsystem.ControlDataFrame(new Vector2(0, .75), 0)));
        });
        autoMode.untilCondition(() -> new TimeDelayCondition(2, TimeUnit.SECONDS)); //set distance fwd after vision and strafing
        autoMode.setNext("auto_part_6");

        //6th step - bk (only for scenario 0)
        autoMode = robot.getOpManager().getOpMode("auto_part_6");
        autoMode.onInit(() -> {
            driveTrain.bind(Data.constant(new MecanumSubsystem.ControlDataFrame(new Vector2(0, -.75), 0)));
        });
        autoMode.untilCondition(() -> new TimeDelayCondition(2, TimeUnit.SECONDS));
        autoMode.setNext("auto_part_7");

        //7th step - turn (only for scenario 0)
        autoMode = robot.getOpManager().getOpMode("auto_part_7");
        autoMode.onInit(() -> {
            driveTrain.bind(Data.constant(new MecanumSubsystem.ControlDataFrame(new Vector2(0, 0), .75)));
        });
        autoMode.untilCondition(() -> new TimeDelayCondition(2, TimeUnit.SECONDS)); //angle to line
        autoMode.setNext("auto_part_8");

        //8th step - fwd (only for scenario 0)
        autoMode = robot.getOpManager().getOpMode("auto_part_6");
        autoMode.onInit(() -> {
            driveTrain.bind(Data.constant(new MecanumSubsystem.ControlDataFrame(new Vector2(0, .75), 0)));
        });
        autoMode.untilCondition(() -> new TimeDelayCondition(2, TimeUnit.SECONDS)); //fwd dist to line
        autoMode.setNext("auto_part_7");
*/

    }
}
