public class Boid {
    Vec2 pos;
    Vec2 vel;
    Vec2 acc;
    float r;
    float maxForce;
    float maxSpeed;
    
    public Boid(Vec2 pos) {
        this.pos = pos;


        vel = new Vec2();
        acc = new Vec2(0,0);        

        r = 2.0;
        maxForce = 0.03;
        maxSpeed = 2;
    }

    public run(Boid[] boids){
        flock(boids);
        update();
        borders();
        render();
    }

    public flock(Boids[] boids){
        
    }
}