package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous(name="Data Collection Auto", group="Auto")
public class DataCollectionAutomated extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();
        bot.odometer.startTracking(0,0,0);

        File f = AppUtil.getInstance().getSettingsFile("DataCollection.txt");

        try {
            FileWriter writer = new FileWriter(f, true);
            writer.append("New Run: \n");

            bot.movement.setTargets(0, 100, 0);
            while(opModeIsActive()) {
                bot.update();

                if(gamepad1.a) {
                    bot.movement.setTargets(0, 0, 0);
                }else if(gamepad1.b) {
                    bot.movement.setTargets(100, 100, 0);
                }else if(gamepad1.x) {
                    bot.movement.setTargets(50, 50, 90);
                }else if(gamepad1.y) {
                    bot.movement.setTargets(100, 0, 90);
                }

                if(bot.time.state == State.CONVERGED) {
                    // LOG
                    writer.append("0\t" + bot.odometer.heading + "\t0\t" + bot.odometer.x + "\t0\t" + bot.odometer.y + "\n");
                    writer.flush();
                    bot.time.delay(20);

                    telemetry.addData("X", bot.odometer.x);
                    telemetry.addData("Y", bot.odometer.y);
                    telemetry.addData("Heading", bot.odometer.heading);
                    telemetry.update();
                }
            }
            writer.append("End Run: \n");
            writer.close();
        }catch (IOException e) {
            telemetry.addData("status","failed to open file");
            telemetry.update();
        }
        bot.drivebase.freeze();
        bot.drivebase.update();
    }

    private void initialize() {
        bot.initialize(hardwareMap);
        telemetry.addData("status","initialized");
        telemetry.update();
    }
}
