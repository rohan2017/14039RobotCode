package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robots.MecanumChassisBot;
import org.firstinspires.ftc.teamcode.Subsystems.State;
import java.io.File;
import java.io.IOException;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name="Data Collection", group="TeleOp")
public class DataCollection extends LinearOpMode {

    // Declare OpMode Members
    private MecanumChassisBot bot = new MecanumChassisBot(this);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("status","running");
        telemetry.update();
        bot.odometer.startTracking(0,0,0);

        boolean start = false;
        boolean end = false;

        SimpleDateFormat sdf = new SimpleDateFormat("yyyy.MM.dd G 'at' HH:mm:ss z");
        String currentDateandTime = sdf.format(new Date());

        File f = AppUtil.getInstance().getSettingsFile("DataCollection.txt");

        try {
            FileWriter writer = new FileWriter(f, true);
            writer.append("New Run: " + currentDateandTime + "\n");

            while(opModeIsActive()) {
                double y1 = -gamepad1.right_stick_y;
                double x1 = gamepad1.right_stick_x;
                double x2 = gamepad1.left_stick_x;
                double y2 = -gamepad1.left_stick_y;

                bot.drivebase.setPowers(y2+x2, y1-x1, y2-x2, y1+x1);
                bot.drivebase.update();
                bot.odometer.update();
                bot.time.update();

                if(gamepad1.start) {
                    start = true;
                }else if(gamepad1.back) {
                    start = false;
                }

                if(start && !end && bot.time.state == State.CONVERGED) {
                    // LOG
                    writer.append("0\t" + bot.odometer.heading + "\t0\t" + bot.odometer.x + "\t0\t" + bot.odometer.y + "\n");
                    writer.flush();
                    bot.time.delay(20);
                    telemetry.addData("status", "logging");
                }
                telemetry.addData("X", bot.odometer.x);
                telemetry.addData("Y", bot.odometer.y);
                telemetry.addData("Heading", bot.odometer.heading);
                telemetry.update();

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