package org.firstinspires.ftc.teamcode;
import java.util.concurrent.TimeUnit;
import java.util.LinkedList;
import java.util.Queue;

public class HydraController {
    public enum State {
        Idle,
        TaskRunning,
        Done;
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
        public String toString(){
            return "Delay(" + delayMs + ")";
        }
    }

    public HydraController delay(long ms){
        tasks.add(new DelayTask(ms));
        return this;
    }
}
