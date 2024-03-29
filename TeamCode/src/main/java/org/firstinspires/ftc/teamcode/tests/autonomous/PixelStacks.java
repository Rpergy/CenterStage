package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.Arrays;


@Autonomous(group = "tests", name = "PixelStacks")
public class PixelStacks extends LinearOpMode {
   public void runOpMode() {
       Actuation.setup(hardwareMap, telemetry);
       telemetry.addData("top", Arrays.toString(Actuation.getColorTop()));
       telemetry.addData("bottom", Arrays.toString(Actuation.getColorBottom()));
       telemetry.update();
       Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[0]);

       Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[5]);
       Trajectory goOver = new Trajectory()
               .lineTo(new Pose(-37.51, 5.75, Math.toRadians(0)), 0.5, 0.5);
       goOver.run();
       Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[3]);
       Trajectory goBack = new Trajectory()
               .lineTo(new Pose(-22, 5.75, Math.toRadians(0)), 0.5, 0.5)
               .lineTo(new Pose(-35.5, 5.75, Math.toRadians(0)), 0.5, 0.5);
       goBack.run();
       double start = System.currentTimeMillis();
       while(System.currentTimeMillis()-start<5000) {
           Actuation.setIntakeArm(ActuationConstants.Intake.stackPos[1]);
           if (Actuation.getColorTop()[0] <= 255 && Actuation.getColorBottom()[0]<=255) {
               Actuation.setIntake(1);
           }
           else {
               Actuation.setIntake(-1);
               Actuation.toggleDeposit(1);
           }
       }
       //Actuation.setIntake(0);
       Trajectory retreat = new Trajectory()
               .lineTo(new Pose(-12, 5.75, Math.toRadians(0)), 0.5, 0.5);
       retreat.run();
   }
}
