package org.firstinspires.ftc.teamcode;
import java.util.concurrent.TimeUnit;
import java.util.LinkedList;
import java.util.Queue;

public class BessieController {
    private Bessie bessie;
    public enum State {
        Idle,
        TaskRunning,
        Done;
    }

    public BessieController(Bessie bessie){
        this.bessie = bessie;
    }

    private interface Task {
        public void begin();
        public boolean isComplete();
    }

    public State state = State.Idle;

    public Queue<Task> tasks = new LinkedList<>();
    public Task currentTask = null;
    public boolean isIdle(){
        return state == State.Idle;
    }
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

    public BessieController delay(long ms){
        tasks.add(new DelayTask(ms));
        return this;
    }

    private class WaitWhileMovingTask implements Task{
        public void begin() {}
        public boolean isComplete(){
            return !bessie.isMoving();
        }
    }

    public BessieController waitWhileMoving(){
        tasks.add(new WaitWhileMovingTask());
        return this;
    }

    private class LowPower implements Task{
        public void begin() {bessie.fwdPower = 0.25f;}
        public boolean isComplete(){
            return true;
        }
    }

    public BessieController lowPower(){
        tasks.add(new LowPower());
        return this;
    }

    private class HighPower implements Task{
        public void begin() {bessie.fwdPower = 0.6f;}
        public boolean isComplete(){
            return true;
        }
    }


    public BessieController highPower(){
        tasks.add(new HighPower());
        return this;
    }

    private class StrafeByTask implements  Task{
        private float distance;
        StrafeByTask(float inches){distance = inches;}
        public void begin(){bessie.strafeBy(distance);}
        public boolean isComplete(){
            return !bessie.isMoving();
        }
    }

    public BessieController strafeBy(float distance){
        tasks.add(new StrafeByTask(distance));
        return this;
    }

    private class ForwardByTask implements Task{
        private float distance;
        ForwardByTask(float inches){distance = inches;}
        public void begin(){bessie.forwardBy(distance);}
        public boolean isComplete(){
            return !bessie.isMoving();
        }
    }

    public BessieController forwardBy(float distance){
        tasks.add(new ForwardByTask(distance));
        return this;
    }

    private class ForwardByAsyncTask implements Task{
        private float distance;
        ForwardByAsyncTask(float inches){distance = inches;}
        public void begin(){bessie.forwardBy(distance);}
        public boolean isComplete() {
            return true;
        }
    }

    public BessieController forwardByAsync(float distance){
        tasks.add(new ForwardByAsyncTask(distance));
        return this;
    }

    private class TurnToTask implements Task{
        private float heading;
        TurnToTask(float radians){heading = radians;}
        public void begin(){bessie.turnTo(heading);}
        public boolean isComplete(){
            return !bessie.isMoving();
        }
    }

    public BessieController turnTo(float heading){
        tasks.add(new TurnToTask(heading));
        return this;
    }

    private class TurnToAsyncTask implements Task{
        private float heading;
        TurnToAsyncTask(float radians){heading = radians;}
        public void begin(){bessie.turnTo(heading);}
        public boolean isComplete(){
            return true;
        }
    }

    public BessieController turnToAsync(float heading){
        tasks.add(new TurnToAsyncTask(heading));
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

