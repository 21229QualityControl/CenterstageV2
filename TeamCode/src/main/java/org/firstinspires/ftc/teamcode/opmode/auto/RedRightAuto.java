package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Red Right Auto", group = "Auto", preselectTeleOp = "Manual Drive")
class RedRightAuto extends AutoBase {
   public static Pose2d pixel;

   @Override
   protected Pose2d getStartPose() {
      return new Pose2d(-64, -36, Math.toRadians(-90));
   }

   @Override
   protected void printDescription() {
      telemetry.addData("Description", "Red Right Auto");
   }

   @Override
   protected void onRun() {
      Actions.runBlocking(


      );
   }
}
