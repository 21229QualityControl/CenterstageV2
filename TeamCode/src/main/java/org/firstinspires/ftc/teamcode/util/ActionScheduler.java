package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.LinkedList;
import java.util.Queue;

public class ActionScheduler {
   final Queue<Action> actions = new LinkedList<>();
   final Queue<Action> parallelActions = new LinkedList<>();
   final FtcDashboard dash = FtcDashboard.getInstance();
   final Canvas canvas = new Canvas();

   public void queueAction(Action action) {
      if (actions.peek() == null) {
         action.preview(canvas);
      }
      actions.add(action);
   }

   public void queueActionParallel(Action action) {
      if (actions.peek() == null) {
         action.preview(canvas);
      }
      parallelActions.add(action);
   }

   public void cancelParallel() {
      parallelActions.clear();
   }

   public void update() {
      TelemetryPacket packet = new TelemetryPacket();
      packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

      if (actions.peek() != null) {
         boolean running = actions.peek().run(packet);
         dash.sendTelemetryPacket(packet);

         if (!running) {
            actions.remove();
            if (actions.peek() != null) {
               actions.peek().preview(canvas);
            }
         }
      }

      if (parallelActions.peek() != null) {
         boolean running = parallelActions.peek().run(packet);
         dash.sendTelemetryPacket(packet);

         if (!running) {
            parallelActions.remove();
            if (parallelActions.peek() != null) {
               parallelActions.peek().preview(canvas);
            }
         }
      }
   }
}
