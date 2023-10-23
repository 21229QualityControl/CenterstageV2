package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.LinkedList;
import java.util.Queue;

public class AutoActionScheduler {
   final Queue<Action> actions = new LinkedList<>();
   final FtcDashboard dash = FtcDashboard.getInstance();
   final Canvas canvas = new Canvas();
   final Runnable pidUpdate;

   public AutoActionScheduler(Runnable pidUpdate) {
      this.pidUpdate = pidUpdate;
   }

   public void addAction(Action action) {
      actions.add(action);
   }

   public void run() {
      while (actions.peek() != null && !Thread.currentThread().isInterrupted()) {
         TelemetryPacket packet = new TelemetryPacket();
         packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

         pidUpdate.run();

         boolean running = actions.peek().run(packet);
         dash.sendTelemetryPacket(packet);

         if (!running) {
            actions.remove();
            if (actions.peek() != null) {
               actions.peek().preview(canvas);
            }
         }
      }
   }
}
