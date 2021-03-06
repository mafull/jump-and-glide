import java.util.regex.*;
import processing.serial.*;

Serial serial;
String inString = "";
int lf = 10;

int rangeX = 1;
int valX = 0;
int rangeY = 1;
int valY = 0;

void setup() {
  size(800, 800);
  
  printArray(Serial.list());
  serial = new Serial(this, Serial.list()[0], 115200);
  serial.clear();  
  serial.bufferUntil(lf);
}


void draw() {
  background(52);
  
  // Parse inString
  //Pattern pattern = Pattern.compile("(-*\\d+) \\((\\d+)\\)");
  Pattern pattern = Pattern.compile("(-*\\d+) \\((\\d+)\\) (-*\\d+) \\((\\d+)\\)");
  Matcher matcher = pattern.matcher(inString);
  if(matcher.find()) {
    text(matcher.group(0), 10, 20);
    
    valX = Integer.parseInt(matcher.group(1));
    int prevRangeX = rangeX;
    rangeX = Integer.parseInt(matcher.group(2));
    if(rangeX <= 1) rangeX = prevRangeX;
    
    valY = Integer.parseInt(matcher.group(3));
    int prevRangeY = rangeY;
    rangeY = Integer.parseInt(matcher.group(4));
    if(rangeY <= 1) rangeY = prevRangeY;
    
    text("MaxX: " + matcher.group(2), 10, 45);
    text("MaxY: " + matcher.group(4), 10, 70);
    
    
  } else {
    text(inString, 10, 20);
  }
  
  stroke(255);
  strokeWeight(2);
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);
  
  
  float xPos = (width/2) + ((width/2)*((float)valX/rangeX));
  float yPos = (height/2) - ((height/2)*((float)valY/rangeY));
  ellipse(xPos, yPos, 20, 20);
}


void serialEvent(Serial s) {
   inString = s.readString();
}