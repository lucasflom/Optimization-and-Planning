//Inverse Kinematics
//CSCI 5611 IK [Solution]
// Stephen J. Guy <sjguy@umn.edu>

/*
INTRODUCTION:
Rather than making an artist control every aspect of a characters animation, we will often specify 
key points (e.g., center of mass and hand position) and let an optimizer find the right angles for 
all of the joints in the character's skelton. This is called Inverse Kinematics (IK). Here, we start 
with some simple IK code and try to improve the results a bit to get better motion.


CHALLENGE:

1. Go back to the 3-limb arm, can you make it look more human-like. Try adding a simple body to 
   the scene using circles and rectangles. Can you make a scene where the character picks up 
   something and moves it somewhere?
2. Create a more full skeleton. How do you handle the torso having two different arms?

*/

void setup(){
  size(640,480);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

//Root
Vec2 root = new Vec2(0,0);

// IKChain arm = new IKChain[3];

//Upper Arm
float l0 = 135; 
float a0 = 0.3; //Shoulder joint
// arm[2] = new IKChain(l0, a0);

//Lower Arm
float l1 = 120;
float a1 = 0.3; //Elbow joint
// arm[1] = new IKChain(l1, a1);

//Hand
float l2 = 75;
float a2 = 0.3; //Wrist joint
// arm[0] = new IKChain(l2, a2);


Vec2 start_l1,start_l2,endPoint;

void solve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  // //Update the finger joint
  // startToGoal = goal.minus(start_l3);
  // startToEndEffector = endPoint.minus(start_l3);
  // dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  // dotProd = clamp(dotProd,-1,1);
  // angleDiff = acos(dotProd);
  // if (cross(startToGoal,startToEndEffector) < 0) 
  //   a3 += angleDiff*0.6;
  // else 
  //   a3 -= angleDiff*0.6;
  // /* Finger joint limits to 0 to 90 */
  // if (a3 < -0.4) a3 = -0.4;
  // if (a3 > 1.9708) a3 = 1.9708;
  // fk(); //Update link positions with fk (e.g. end effector changed)

  //Update wrist joint
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2 += angleDiff*0.6;
  else
    a2 -= angleDiff*0.6;
  /* Wrist joint limits to 90 degrees */
  if (abs(a2) >= 1.5708){
    if (a2 < 0) a2 = -1.5708;
    else a2 = 1.5708;
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  
  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1 += angleDiff*0.5;
  else
    a1 -= angleDiff*0.5;
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0 += angleDiff*0.4;
  else
    a0 -= angleDiff*0.4;
  /*Shoulder joint limits from 0 to 90 degrees*/
  if (a0 < 0) a0 = 0;
  if (a0 > 1.5708) a0 = 1.5708;
  fk(); //Update link positions with fk (e.g. end effector changed)
 
  // println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2);
}

void fk(){
  // arm[0].fk(root);
  // IKChain[] temp = {arm[0]};
  // for (int i = 1; i < arm.length; i++) {
  //   arm[i].fk(temp);
  //   temp = splice(temp,arm[i],i);
  // }
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  endPoint = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);

}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(250,250,250);
  

  fill(255,219,172);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  
  // pushMatrix();
  // translate(start_l3.x,start_l3.y);
  // rotate(a0+a1+a2+a3);
  // rect(0, -armW/4, l3, armW/2);
  // popMatrix();

}

