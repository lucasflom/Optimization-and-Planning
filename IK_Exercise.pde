//Inverse Kinematics
//CSCI 5611 IK [Solution]
// Stephen J. Guy <sjguy@umn.edu>

/*
CHALLENGE:

1. Go back to the 3-limb arm, can you make it look more human-like. Try adding a simple body to 
   the scene using circles and rectangles. Can you make a scene where the character picks up 
   something and moves it somewhere?
2. Create a more full skeleton. How do you handle the torso having two different arms?

*/

void setup(){
  size(1280,960);
  root = new Vec2((bodyW/2) + (width/2), (height/2));
  surface.setTitle("Inverse Kinematics Part 1");
  // Set the Body
  body = new Box(width/2, height/2, bodyH, bodyW);
  head = new Circle(new Vec2(width/2, (height/2)-(30 + (bodyH/2))), 30);
}

//Root
Vec2 root;
Box[] arm = new Box[4];

//Upper Arm
float l0 = 90; 
float a0 = 0.3; //Shoulder joint

//Lower Arm
float l1 = 80;
float a1 = 0.3; //Elbow joint

//Hand
float l2 = 50;
float a2 = 0.3; //Wrist joint

//Finger
float l3 = 20;
float a3 = 0.3; //Finger joint

Vec2 start_l1,start_l2,start_l3,endPoint;

void solve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //Update finger joint
  float old_a3 = a3;
  startToGoal = goal.minus(start_l3);
  startToEndEffector = endPoint.minus(start_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  do {
    a3 = old_a3;
    if (cross(startToGoal,startToEndEffector) < 0)
      a3 += angleDiff*0.4;
    else
      a3 -= angleDiff*0.4;
    /* Finger joint limits to 90 degrees */
    if (a3 < -1.5708) a3 = -1.5708;
    if (a3 > 1.5708) a3 = 1.5708;
    fk(); //Update link positions with fk (e.g. end effector changed)
    angleDiff *= 0.5;
    println(a3, old_a3);
    
  } while (head.isColliding(arm[3]));
  




  //Update wrist joint
  float old_a2 = a2;
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  do {
    a2 = old_a2;
    if (cross(startToGoal,startToEndEffector) < 0)
      a2 += angleDiff*0.4;
    else
      a2 -= angleDiff*0.4;
    /* Wrist joint limits to 90 degrees */
    if (a2 < -1.5708) a2 = -1.5708;
    if (a2 > 1.5708) a2 = 1.5708;
    fk(); //Update link positions with fk (e.g. end effector changed)
    angleDiff *= 0.5;
  } while (head.isColliding(arm[2]));
  
  
    
  
  
  //Update elbow joint
  float old_a1 = a1;
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  do {
    a1 = old_a1;
    if (cross(startToGoal,startToEndEffector) < 0)
      a1 += angleDiff*0.3;
    else
      a1 -= angleDiff*0.3;
    /* Elbow joint limits to 90 degrees */
    if (a1 < -1.5708) a1 = -1.5708;
    if (a1 > 1.5708) a1 = 1.5708;
    fk(); //Update link positions with fk (e.g. end effector changed)
    angleDiff *= 0.5;
  } while (head.isColliding(arm[1]));
  
  
  //Update shoulder joint
  float old_a0 = a0;
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  do {
    a0 = old_a0;
    if (cross(startToGoal,startToEndEffector) < 0)
      a0 += angleDiff*0.2;
    else
      a0 -= angleDiff*0.2;
    /*Shoulder joint limits from 0 to 90 degrees*/
    if (a0 < -1.5708) a0 = -1.5708;
    if (a0 > 1.5708) a0 = 1.5708;
    fk(); //Update link positions with fk (e.g. end effector changed)
    angleDiff *= 0.5;
  } while (head.isColliding(arm[0]));
 
}
Vec2 closest = new Vec2(0,0);
void fk(){

  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  arm[0] = new Box(start_l1.x, start_l1.y, armW, l0);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  arm[1] = new Box(start_l2.x, start_l2.y, armW, l1);
  start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
  arm[2] = new Box(start_l3.x, start_l3.y, armW, l2);
  endPoint = new Vec2(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3).plus(start_l3);
  arm[3] = new Box(endPoint.x, endPoint.y, armW/2, l3);

}

float armW = 20;
float bodyW = 80;
float bodyH = 160;
Box body;
Circle head;

void draw(){
  fk();
  solve();
  background(250,250,250);
  // Makes the Arm
  rectMode(CORNER);
  fill(255,219,172);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  circle(0, 0, 15);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  circle(0, 0, 15);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, l2, armW);
  circle(0, 0, 15);
  popMatrix();

  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(a0+a1+a2+a3);
  rect(0, -armW/4, l3, armW/2);
  popMatrix();
  
  // Makes the Body
  rectMode(CENTER);
  // pushMatrix();
  // translate(width/2, height/2);
  // rect(0,0, bodyW, bodyH);
  // circle(0,-(30 + (bodyH/2)),60);
  // popMatrix();
  rect(body.pos.x, body.pos.y, body.h, body.w);
  circle(head.pos.x, head.pos.y, head.r*2);
  fill(255,0,0);
  circle(arm[3].pos.x, arm[3].pos.y,10);
  fill(0,255,0);
  circle(closest.x, closest.y, 10);
}

