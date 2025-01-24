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
        slideTgt = 350;
        nextSlideTgt = 4500;
        nextPivotTgt = 1100;
        state = State_StowSlide;
    }
    public void moveToTravel(){
        if (state!= State_Inital) return;
        slideTgt = 300;
        state= State_StowSlide;
        nextSlideTgt = 300;
        nextPivotTgt = 500;
    }
    public void moveToPick(){
        if (state!= State_Inital) return;
        slideTgt = 350;
        state= State_StowSlide;
        nextSlideTgt = 2050;
        nextPivotTgt = 400;
    }

    public boolean isMoving(){
        float slidepos = hydra.slide.getCurrentPosition();
        if(state == State_Inital) {

            if (Math.abs((slideTgt - slidepos) / slidepos) < 0.03) {
                //slide is near enough to position we can assume its not moving
                return false;
            } else {
                //slide position is not near its target, assume movement
                return true;
            }
        } else{
            //isMoving = true
            return true;
        }
    }

    public void update(){
        if(state == State_Inital){

        }
        else if (state == State_StowSlide) {
            float slidepos = hydra.slide.getCurrentPosition();
            if (slidepos == 0)
                slidepos = 0.01f;
             if( Math.abs((slideTgt-slidepos)/slidepos)< 0.03){
                // we're considering the task complete.
                 state = State_Rotate;
                 pivotTgt = nextPivotTgt;
             }
        }
        else if (state == State_Rotate) {
            float pivotpos = hydra.slideTurner.getCurrentPosition();
            if (pivotpos == 0)
                pivotpos = 0.01f;
            if (Math.abs((pivotTgt-pivotpos) / pivotpos) < 0.03) {
                // were considering the task complete.
                state = State_Inital;
                slideTgt = nextSlideTgt;
            }
        }
        else if (state == State_MoveHigh) {
               boolean slideComplete = false;
                float slidepos = hydra.slide.getCurrentPosition();
                if (slidepos == 0)
                    slidepos = 0.01f;
                if( Math.abs((slideTgt-slidepos)/slidepos)< 0.03) {
               slideComplete = true;
                }
                float pivotpos = hydra.slideTurner.getCurrentPosition();
                if (pivotpos == 0)
                    pivotpos = 0.01f;
                if(slideComplete && Math.abs((pivotTgt-pivotpos)/pivotpos)< 0.03){
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
        pivotTgt = hydra.clamp(pivotTgt,0,1100);
        float pivotError = pivotTgt - hydra.slideTurner.getCurrentPosition();
        pivotError = hydra.clamp(pivotError/200.0f,-0.5f,0.7f);
        hydra.slideTurner.setPower(pivotError);
    }
}
