//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Exercise]
// Stephen J. Guy <sjguy@umn.edu>

/*
TODO:    
Challenge:
  2. Let the user use the mouse to click and drag the obstacles.
*/


//A list of circle obstacles
static int numObstacles = 50;
Vec2 circlePos[] = new Vec2[numObstacles]; //Circle positions
float circleRad[] = new float[numObstacles];  //Circle radii

//A box obstacle
// Vec2 boxTopLeft = new Vec2(100,100);
// float boxW = 100;
// float boxH = 250;
//A list of box obstacles
static int numObstaclesBox = 20;
Vec2 boxTopLefts[] = new Vec2[numObstaclesBox];
float boxWs[] = new float[numObstaclesBox];
float boxHs[] = new float[numObstaclesBox];

Vec2 startPos = new Vec2(100,500);
Vec2 goalPos = new Vec2(500,200);


void placeRandomObstacles(){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3));
  }
  for (int i = 0; i < numObstaclesBox; i++){
    boxTopLefts[i] = new Vec2(random(50,width-50), random(50,height-50));
    boxWs[i] = random(10, 100);
    boxHs[i] = random(10, 100);
  }
}

int strokeWidth = 2;
void setup(){
  size(1024,768);
  placeRandomObstacles();
  buildPRM(circlePos, circleRad, boxTopLefts, boxWs, boxHs);
  runBFS(closestNode(startPos),closestNode(goalPos));
}

void draw(){
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(200); //Grey background
  stroke(0,0,0);
  fill(255,255,255);
  
  
  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    circle(c.x,c.y,r*2);
  }
  
  //Draw the box obstacles
  fill(250,200,200);
  for (int i = 0; i < numObstaclesBox; i++){
    rect(boxTopLefts[i].x, boxTopLefts[i].y, boxWs[i], boxHs[i]);
  }
  // rect(boxTopLeft.x, boxTopLeft.y, boxW, boxH);
  
  //Draw PRM Nodes
  fill(0);
  for (int i = 0; i < numNodes; i++){
    circle(nodePos[i].x,nodePos[i].y,5);
  }
  
  //Draw graph
  stroke(100,100,100);
  strokeWeight(1);
  for (int i = 0; i < numNodes; i++){
    for (int j : neighbors[i]){
      line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
    }
  }
  
  //Draw Start and Goal
  fill(20,60,250);
  circle(nodePos[startNode].x,nodePos[startNode].y,20);
  //circle(startPos.x,startPos.y,20);
  fill(250,30,50);
  circle(nodePos[goalNode].x,nodePos[goalNode].y,20);
  //circle(goalPos.x,goalPos.y,20);
  
  //Draw Planned Path
  stroke(20,255,40);
  strokeWeight(5);
  for (int i = 0; i < path.size()-1; i++){
    int curNode = path.get(i);
    int nextNode = path.get(i+1);
    line(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
  }
  
}

void keyPressed(){
  if (key == 'r'){
    placeRandomObstacles();
    buildPRM(circlePos, circleRad, boxTopLefts, boxWs, boxHs);
    runBFS(closestNode(startPos),closestNode(goalPos));
  }
}

int closestNode(Vec2 point){
  int winner = -1;
  float bestDist = Float.POSITIVE_INFINITY;
  for (int i = 0; i < numNodes; i++){
    float distBetween = abs(nodePos[i].distanceTo(point));
    if (distBetween < bestDist) {
      bestDist = distBetween;
      winner = i;
    }
  }
  return winner;
}

void mousePressed(){
  goalPos = new Vec2(mouseX, mouseY);
  println("New Goal is",goalPos.x, goalPos.y);
  runBFS(closestNode(startPos),closestNode(goalPos));
}



/////////
// Point Intersection Tests
/////////

//Returns true if the point is inside a box
boolean pointInBox(Vec2 boxTopLeft, float boxW, float boxH, Vec2 pointPos){
  if ((pointPos.x >= boxTopLeft.x - 2 && pointPos.x <= (boxTopLeft.x + boxW + 2)) && (pointPos.y >= boxTopLeft.y - 2 && pointPos.y <= (boxTopLeft.y + boxH + 2))) return true;
  return false;
}

//Returns true if the point is inside a list of boxes
boolean pointInBoxList(Vec2[] boxTopLefts, float[] boxWs, float[] boxHs, Vec2 pointPos){
  for (int i = 0; i < numObstaclesBox; i++){
    Vec2 boxTopLeft = boxTopLefts[i];
    float boxW = boxWs[i];
    float boxH = boxHs[i];
    if (pointInBox(boxTopLeft, boxW, boxH, pointPos)) return true;
  }
  return false;
}

//Returns true if the point is inside a circle
boolean pointInCircle(Vec2 center, float r, Vec2 pointPos){
  float dist = pointPos.distanceTo(center);
  if (dist < r+2){ //small safety factor
    return true;
  }
  return false;
}

//Returns true if the point is inside a list of circle
boolean pointInCircleList(Vec2[] centers, float[] radii, Vec2 pointPos){
  for (int i = 0; i < numObstacles; i++){
    Vec2 center =  centers[i];
    float r = radii[i];
    if (pointInCircle(center,r,pointPos)){
      return true;
    }
  }
  return false;
}




/////////
// Ray Intersection Tests
/////////

class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}

hitInfo rayBoxIntersect(Vec2 boxTopLeft, float boxW, float boxH, Vec2 ray_start, Vec2 ray_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.hit = true;
  
  float t_left_x, t_right_x, t_top_y, t_bot_y;
  t_left_x = (boxTopLeft.x - ray_start.x)/ray_dir.x;
  t_right_x = (boxTopLeft.x + boxW - ray_start.x)/ray_dir.x;
  t_top_y = (boxTopLeft.y - ray_start.y)/ray_dir.y;
  t_bot_y = (boxTopLeft.y + boxH - ray_start.y)/ray_dir.y;
  
  float t_max_x = max(t_left_x,t_right_x);
  float t_max_y = max(t_top_y,t_bot_y);
  float t_max = min(t_max_x,t_max_y); //When the ray exists the box
  
  float t_min_x = min(t_left_x,t_right_x);
  float t_min_y = min(t_top_y,t_bot_y);
  float t_min = max(t_min_x,t_min_y); //When the ray enters the box
  
  
  //The the box is behind the ray (negative t)
  if (t_max < 0){
    hit.hit = false;
    hit.t = t_max;
    return hit;
  }
  
  //The ray never hits the box
  if (t_min > t_max){
    hit.hit = false;
  }
  
  //The ray hits, but further out than max_t
  if (t_min > max_t){
    hit.hit = false;
  }
  
  hit.t = t_min;
  return hit;
}

hitInfo rayBoxListIntersect(Vec2[] boxTopLefts, float[] boxWs, float[] boxHs, Vec2 ray_start, Vec2 ray_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.t = max_t;
  for (int i = 0; i < numObstaclesBox; i++){
    hitInfo boxHit = rayBoxIntersect(boxTopLefts[i], boxWs[i], boxHs[i], ray_start, ray_dir, hit.t);
    if (boxHit.t > 0 && boxHit.t < hit.t){
      hit.hit = true;
      hit.t = boxHit.t;
    } else if (boxHit.hit && boxHit.t < 0){
      hit.hit = true;
      hit.t = -1;
    }
  }
  return hit;
}

hitInfo rayCircleIntersect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  
  //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec2 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Length of l_dir (we normalized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - (r+strokeWidth)*(r+strokeWidth); //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the length of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only need the first collision
      float t2 = (-b + sqrt(d))/(2*a); //Optimization: we only need the first collision
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < max_t){
        hit.hit = true;
        hit.t = t1;
      }
      else if (t1 < 0 && t2 > 0){
        hit.hit = true;
        hit.t = -1;
      }
      
    }
    
  return hit;
}

hitInfo rayCircleListIntersect(Vec2[] centers, float[] radii, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.t = max_t;
  for (int i = 0; i < numObstacles; i++){
    Vec2 center = centers[i];
    float r = radii[i];
    
    hitInfo circleHit = rayCircleIntersect(center, r, l_start, l_dir, hit.t);
    if (circleHit.t > 0 && circleHit.t < hit.t){
      hit.hit = true;
      hit.t = circleHit.t;
    }
    else if (circleHit.hit && circleHit.t < 0){
      hit.hit = true;
      hit.t = -1;
    }
  }
  return hit;
}




/////////////////////////////////
// A Probabilistic Roadmap (PRM)
////////////////////////////////

static int numNodes = 100;

//The optimal path found along the PRM
ArrayList<Integer> path = new ArrayList();
int startNode, goalNode; //The actual node the PRM tries to connect do

//Represent our graph structure as 3 lists
ArrayList<Integer>[] neighbors = new ArrayList[numNodes];  //A list of neighbors can can be reached from a given node
Boolean[] visited = new Boolean[numNodes]; //A list which store if a given node has been visited
int[] parent = new int[numNodes]; //A list which stores the best previous node on the optimal path to reach this node

//The PRM uses the above graph, along with a list of node positions
Vec2[] nodePos = new Vec2[numNodes];

//Generate non-colliding PRM nodes
void generateRandomNodes(Vec2[] circleCenters, float[] circleRadii, Vec2[] boxTopLefts, float[] boxWs, float[] boxHs){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,randPos);
    boolean insideAnyBox = pointInBoxList(boxTopLefts, boxWs, boxHs, randPos);
    while (insideAnyCircle || insideAnyBox){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,randPos);
      insideAnyBox = pointInBoxList(boxTopLefts, boxWs, boxHs, randPos);
    }
    nodePos[i] = randPos;
  }
}


//Set which nodes are connected to which neighbors based on PRM rules
void connectNeighbors(){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(circlePos, circleRad, nodePos[i], dir, distBetween);
      hitInfo boxListCheck = rayBoxListIntersect(boxTopLefts, boxWs, boxHs, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit && distBetween < 200 && !boxListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//Build the PRM
// 1. Generate collision-free nodes
// 2. Connect mutually visible nodes as graph neighbors
void buildPRM(Vec2[] circleCenters, float[] circleRadii, Vec2[] boxTopLefts, float[] boxWs, float[] boxHs){
  generateRandomNodes(circleCenters, circleRadii, boxTopLefts, boxWs, boxHs);
  connectNeighbors();
}

//BFS
void runBFS(int startID, int goalID){
  startNode = startID;
  goalNode = goalID;
  ArrayList<Integer> fringe = new ArrayList();  //Make a new, empty fringe
  path = new ArrayList(); //Reset path
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(startID);
  println("Adding node", startID, "(start) to the fringe.");
  println(" Current Fring: ", fringe);
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (currentNode == goalID){
      println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        println("Added node", neighborNode, "to the fringe.");
        println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  print(goalID, " ");
  while (prevNode >= 0){
    print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  print("\n");
}