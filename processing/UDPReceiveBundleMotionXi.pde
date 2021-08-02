/*
recives OSCBundles

print addrPattern() to see ID
change addrPattern() for right ID
*/

import oscP5.*;
import netP5.*;
  
OscP5 oscP5;

float analogValue0 = 50;
float analogValue1 = 50;
float analogValue2 = 50;

float xOffset = PI/2;
float yOffset = PI/4;
float zOffset = PI;

int Port = 9001;
String devID = "Mot1";

void setup() {
  size(600,400, P3D);
  frameRate(30);
  //set this to the receiving port
  oscP5 = new OscP5(this,Port);
}


void draw() {
  background(0); 
  
  float analog0Height=analogValue0;
  float analog1Height=analogValue1;
  float analog2Height=analogValue2;
  
  fill(255);
  rect(50, 150, 50, -analog0Height);
  rect(150, 150, 50, -analog1Height);
  rect(250, 150, 50, -analog2Height);
  //and the labels
  textSize(12);
  text("/Proto/roll", 50, 270);
  text("/Proto/pitch", 150, 270);
  text("/Proto/yaw", 250, 270);
  
  text(analogValue0, 50, 300);
  text(analogValue1, 150, 300);
  text(analogValue2, 250, 300);
  
  stroke(25, 50);
  translate(450, 100, 0);
  rotateX(radians(analogValue0)+xOffset);
  rotateY(radians(analogValue1)+yOffset);
  rotateZ(radians(analogValue2)+zOffset);
  fill(mouseX * 2, 0, 160);
  sphereDetail(mouseX / 4);
  sphere(40);
}

// incoming osc message are forwarded to the oscEvent method. 
void oscEvent(OscMessage theOscMessage) {
  //println(theOscMessage.addrPattern());
  if (theOscMessage.addrPattern().equals("/"+devID+"/roll")){
    analogValue0 = theOscMessage.get(0).floatValue();
    //println(analogValue0);
  } else if(theOscMessage.addrPattern().equals("/"+devID+"/pitch")){
    analogValue1 = theOscMessage.get(0).floatValue();
    //println(analogValue1);
  } else if(theOscMessage.addrPattern().equals("/"+devID+"/yaw")){
    analogValue2 = theOscMessage.get(0).floatValue();
    //println(analogValue2);
  }
}
