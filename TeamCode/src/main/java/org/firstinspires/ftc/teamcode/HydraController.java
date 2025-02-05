package org.firstinspires.ftc.teamcode;
import java.util.concurrent.TimeUnit;
import java.util.LinkedList;
import java.util.Queue;

public class HydraController {
    private Hydra hydra;
    public enum State {
        Idle,
        TaskRunning,
        Done;
    }

    public HydraController(Hydra hydra){
        this.hydra = hydra;
    }

    private interface Task {
        public void begin();
        public boolean isComplete();
    }

    public State state = State.Idle;

    public Queue<Task> tasks = new LinkedList<>();
    public Task currentTask = null;

    public void update(){
        switch (state){
            case Idle:
                //find next command to run
                if (tasks.isEmpty()){
                    state = State.Done;
                }
                else{
                    currentTask = tasks.remove();
                    state = State.TaskRunning;
                    currentTask.begin();
                }
                break;
            case TaskRunning:
                if(currentTask.isComplete())
                    state = State.Idle;
                break;
            case Done:
                break;
        }
    }

    private class DelayTask implements Task {
        private long endTime;
        private long delayMs;
        DelayTask(long delayMs){
            this.delayMs = delayMs;
        }
        public void begin(){
            endTime = System.currentTimeMillis() + delayMs;
        }
        public boolean isComplete(){
            return endTime <= System.currentTimeMillis();
        }
     }

    public HydraController delay(long ms){
        tasks.add(new DelayTask(ms));
        return this;
    }

    private class WaitWhileMovingTask implements Task{
        public void begin() {}
        public boolean isComplete(){
            return !hydra.isMoving();
        }
    }

    public HydraController waitWhileMoving(){
        tasks.add(new WaitWhileMovingTask());
        return this;
    }

    private class StrafeByTask implements  Task{
        private float distance;
        StrafeByTask(float inches){distance = inches;}
        public void begin(){hydra.strafeBy(distance);}
        public boolean isComplete(){
            return !hydra.isMoving();
        }
    }

    public HydraController strafeBy(float distance){
        tasks.add(new StrafeByTask(distance));
        return this;
    }

     private class ForwardByTask implements Task{
        private float distance;
        ForwardByTask(float inches){distance = inches;}
        public void begin(){hydra.forwardBy(distance);}
        public boolean isComplete(){
            return !hydra.isMoving();
        }
     }

    public HydraController forwardBy(float distance){
        tasks.add(new ForwardByTask(distance));
        return this;
    }

    private class ForwardByAsyncTask implements Task{
        private float distance;
        ForwardByAsyncTask(float inches){distance = inches;}
        public void begin(){hydra.forwardBy(distance);}
        public boolean isComplete() {
            return true;
        }
    }

        public HydraController forwardByAsync(float distance){
            tasks.add(new ForwardByAsyncTask(distance));
            return this;
        }

    private class TurnToTask implements Task{
        private float heading;
        TurnToTask(float radians){heading = radians;}
        public void begin(){hydra.turnTo(heading);}
        public boolean isComplete(){
            return !hydra.isMoving();
        }
    }

    public HydraController turnTo(float heading){
        tasks.add(new TurnToTask(heading));
        return this;
    }

    private class TurnToAsyncTask implements Task{
        private float heading;
        TurnToAsyncTask(float radians){heading = radians;}
        public void begin(){hydra.turnTo(heading);}
        public boolean isComplete(){
            return true;
        }
    }

    public HydraController turnToAsync(float heading){
        tasks.add(new TurnToAsyncTask(heading));
        return this;
    }

    private class RotateArmTask implements Task{
        private float x;
        RotateArmTask(float x){this.x = x;}
        public void begin(){hydra.arm.rotate(x);}
        public boolean isComplete(){
            return true;
        }
    }

    public HydraController rotateArmBy(float x){
        tasks.add(new RotateArmTask(x));
        return this;
    }

    private class MoveArmToTravelTask implements Task{
        public void begin(){hydra.arm.moveToTravel();}
        public boolean isComplete(){
            return !hydra.arm.isMoving();
        }
    }

    public HydraController moveArmToTravel(){
        tasks.add(new MoveArmToTravelTask());
        return this;
    }

    private class MoveArmToHighTask implements Task{
        public void begin(){hydra.arm.moveToHigh();}
        public boolean isComplete(){
            return !hydra.arm.isMoving();
        }
    }

    public HydraController moveArmToHigh(){
        tasks.add(new MoveArmToHighTask());
        return this;
    }

    private class MoveArmToPickTask implements Task{
        public void begin(){hydra.arm.moveToPick();}
        public boolean isComplete(){
            return !hydra.arm.isMoving();
        }
    }

    public HydraController moveArmToPick(){
        tasks.add(new MoveArmToPickTask());
        return this;
    }

    private class OpenClawTask implements Task{
        public void begin(){hydra.openClaw();}
        public boolean isComplete(){
            return true;
        }
    }

    public HydraController openClawTask(){
        tasks.add(new OpenClawTask());
        return this;
    }

    private class CloseClawTask implements Task{
        public void begin(){hydra.closeClaw();}
        public boolean isComplete(){
            return true;
        }
    }

    public HydraController closeClawTask(){
        tasks.add(new CloseClawTask());
        return this;
    }

    public String toString(){
        if(currentTask != null) {
            return state.toString() + ":" + currentTask.toString();
        } else{
            return state.toString();
        }
    }
}
