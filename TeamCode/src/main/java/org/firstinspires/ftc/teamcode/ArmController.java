package org.firstinspires.ftc.teamcode;

public class ArmController {
    final int State_Inital =0;
    final int State_StowSlide =1;
    final int State_Rotate =2;
    final int State_MoveHigh =3;
    int state = State_Inital;
    float pivotTgt = 0;
    float slideTgt = 0;
    Hydra hydra;
    float nextPivotTgt;
    float nextSlideTgt;
    ArmController(Hydra hydra){
        this.hydra = hydra;
    }
    public void extendSlide(float x){
        if (state!= State_Inital) return;
        slideTgt += x;
    }
    public void rotate(float x){
        if (state!= State_Inital) return;
        pivotTgt += x;
    }
    public void moveToHigh(){
        if (state!= State_Inital) return;
        slideTgt = 4500;
        pivotTgt = 1230;
        state = State_MoveHigh;
    }
    public void moveToTravel(){
        if (state!= State_Inital) return;
        slideTgt = 500;
        state= State_StowSlide;
        nextSlideTgt = 500;
        nextPivotTgt = 500;
    }
    public void moveToPick(){
        if (state!= State_Inital) return;
        slideTgt = 500;
        state= State_StowSlide;
        nextSlideTgt = 2050;
        nextPivotTgt = 400;
    }
    public void update(){
        if(state == State_Inital){

        }
        else if (state == State_StowSlide) {
            float slidepos = hydra.slide.getCurrentPosition();
            if (slidepos == 0)
                slidepos = 0.01f;
             if( Math.abs(slideTgt/slidepos)< 0.05){
                // were considering the task complete.
                 state = State_Rotate;
                 pivotTgt = nextPivotTgt;
             }
        }
        else if (state == State_Rotate) {
            float pivotpos = hydra.slideTurner.getCurrentPosition();
            if (pivotpos == 0)
                pivotpos = 0.01f;
            if (Math.abs(pivotTgt / pivotpos) < 0.05) {
                // were considering the task complete.
                state = State_Inital;
            }
        }
        else if (state == State_MoveHigh) {
               boolean slideComplete = false;
                float slidepos = hydra.slide.getCurrentPosition();
                if (slidepos == 0)
                    slidepos = 0.01f;
                if( Math.abs(slideTgt/slidepos)< 0.05) {
               slideComplete = true;
                }
                float pivotpos = hydra.slideTurner.getCurrentPosition();
                if (pivotpos == 0)
                    pivotpos = 0.01f;
                if(slideComplete && Math.abs(pivotTgt/pivotpos)< 0.05){
                    // were considering the task complete.
                    state = State_Inital;
                    }
        }
        slideTgt = hydra.clamp(slideTgt,0,4500);
        float slidePos = hydra.slide.getCurrentPosition();
        float slideError = slideTgt - slidePos;
        slideError = hydra.clamp(slideError/100.0f,-1.0f,1.0f);
        hydra.slide.setPower(slideError);

        //pivotTgt += -gamepad2.right_stick_y * 25;
        pivotTgt = hydra.clamp(pivotTgt,0,1500);
        float pivotError = pivotTgt - hydra.slideTurner.getCurrentPosition();
        pivotError = hydra.clamp(pivotError/200.0f,-0.85f,0.85f);
        hydra.slideTurner.setPower(pivotError);
    }
}
