// Maurice Ribble 
// 6-28-2009
// http://www.glacialwanderer.com/hobbyrobotics

// This takes data off the serial port and graphs it.
// There is an option to log this data to a file.

// I wrote an arduino app that sends data in the format expected by this app.
// The arduino app sends accelerometer and gyroscope data.
//
// Modified by Bro_Az August 2010 for PID monitoring.
// Modified by Matthew Tesfaldet March 2013 for compatibility

import processing.serial.*;

// Globals
int g_winW             = 820;   // Window Width
int g_winH             = 600;   // Window Height
boolean g_dumpToFile   = false;  // Dumps data to c:\\output.txt in a comma seperated format (easy to import into Excel)
boolean g_enableFilter = false;  // Enables simple filter to help smooth out data.

cDataArray g_xAccel    = new cDataArray(200);
cDataArray g_yAccel    = new cDataArray(200);
cDataArray g_zAccel    = new cDataArray(200);
cDataArray g_vRef      = new cDataArray(200);
cDataArray g_xRate     = new cDataArray(200);
cDataArray g_yRate     = new cDataArray(200);
cGraph g_graph         = new cGraph(10, 190, 800, 400);
Serial g_serial;
PFont  g_font;

void setup()
{
  size(g_winW, g_winH, P2D);

  println(Serial.list());
  g_serial = new Serial(this, Serial.list()[0], 9600, 'N', 8, 1.0);
  g_font = loadFont("ArialMT-20.vlw");
  textFont(g_font, 20);
  
  // This draws the graph key info
  strokeWeight(1.5);
  stroke(255, 0, 0);     line(20, 420, 35, 420);
  stroke(0, 255, 0);     line(20, 440, 35, 440);
  stroke(0, 0, 255);     line(20, 460, 35, 460);
  stroke(255, 255, 0);   line(20, 480, 35, 480);
  stroke(255, 0, 255);   line(20, 500, 35, 500);
  stroke(0, 255, 255);   line(20, 520, 35, 520);
  fill(0, 0, 0);
  text("Temperature Setpoint", 40, 430);
  text("Actual Temperature", 40, 450);
  text("Controller Output", 40, 470);
  text("Analog 3", 40, 490);
  text("Analog 4", 40, 510);
  text("Analog 5", 40, 530);
  
  if (g_dumpToFile)
  {
    // This clears deletes the old file each time the app restarts
    byte[] tmpChars = {'\r', '\n'};
    saveBytes("c:\\output.txt", tmpChars);
  }
}

void draw()
{
  // We need to read in all the avilable data so graphing doesn't lag behind
  while (g_serial.available() >= 2*6+2)
  {
    processSerialData();
  }

  strokeWeight(1);
  fill(255, 255, 255);
  g_graph.drawGraphBox();
  
  strokeWeight(1.5);
  stroke(255, 0, 0);
  g_graph.drawLine(g_xAccel, 0, 1024);
  stroke(0, 255, 0);
  g_graph.drawLine(g_yAccel, 0, 1024);
  stroke(0, 0, 255);
  g_graph.drawLine(g_zAccel, 0, 1024);
  stroke(255, 255, 0);
  g_graph.drawLine(g_vRef, 0, 1024);
  stroke(255, 0, 255);
  g_graph.drawLine(g_xRate, 0, 1024);
  stroke(0, 255, 255);
  g_graph.drawLine(g_yRate, 0, 1024);
}

// This reads in one set of the data from the serial port
void processSerialData()
{
  int inByte = 0;
  int curMatchPos = 0;
  int[] intBuf = new int[2];

  intBuf[0] = 0xAD;
  intBuf[1] = 0xDE;
  
  while (g_serial.available() < 2); // Loop until we have enough bytes
  inByte = g_serial.read();
  
  // This while look looks for two bytes sent by the client 0xDEAD
  // This allows us to resync the server and client if they ever
  // lose sync.  In my testing I haven't seen them lose sync so
  // this could be removed if you need to, but it is a good way to
  // prevent catastrophic failure.
  while(curMatchPos < 2)
  {
    if (inByte == intBuf[curMatchPos])
    {
      ++curMatchPos;
      
      if (curMatchPos == 2)
        break;
    
      while (g_serial.available() < 2); // Loop until we have enough bytes
      inByte = g_serial.read();
    }
    else
    {
      if (curMatchPos == 0)
      {
        while (g_serial.available() < 2); // Loop until we have enough bytes
        inByte = g_serial.read();
      }
      else
      {
        curMatchPos = 0;
      }
    }
  }
  
  while (g_serial.available() < 2*6);  // Loop until we have a full set of data

  // This reads in one set of data
  {
    byte[] inBuf = new byte[2];
    int xAccel, yAccel, zAccel, vRef, xRate, yRate;
  
    g_serial.readBytes(inBuf);
    // Had to do some type conversion since Java doesn't support unsigned bytes
    xAccel = ((int)(inBuf[1]&0xFF) << 8) + ((int)(inBuf[0]&0xFF) << 0);
    g_serial.readBytes(inBuf);
    yAccel = ((int)(inBuf[1]&0xFF) << 8) + ((int)(inBuf[0]&0xFF) << 0);
    g_serial.readBytes(inBuf);
    zAccel = ((int)(inBuf[1]&0xFF) << 8) + ((int)(inBuf[0]&0xFF) << 0);
    g_serial.readBytes(inBuf);
    vRef   = ((int)(inBuf[1]&0xFF) << 8) + ((int)(inBuf[0]&0xFF) << 0);
    g_serial.readBytes(inBuf);
    xRate  = ((int)(inBuf[1]&0xFF) << 8) + ((int)(inBuf[0]&0xFF) << 0);
    g_serial.readBytes(inBuf);
    yRate  = ((int)(inBuf[1]&0xFF) << 8) + ((int)(inBuf[0]&0xFF) << 0);
    
    g_xAccel.addVal(xAccel);
    g_yAccel.addVal(yAccel);
    g_zAccel.addVal(zAccel);
    g_vRef.addVal(vRef);
    g_xRate.addVal(xRate);
    g_yRate.addVal(yRate);

    if (g_dumpToFile)  // Dump data to a file if needed
    {
      String tempStr;
      tempStr = xAccel + "," + yAccel + "," + zAccel + "," + vRef + "," + xRate + "," + yRate + "\r\n";
      FileWriter file;

      try  
      {  
        file = new FileWriter("c:\\output.txt", true); //bool tells to append
        file.write(tempStr, 0, tempStr.length()); //(string, start char, end char)
        file.close();
      }  
      catch(Exception e)  
      {  
        println("Error: Can't open file!");
      }
    }

    /*
    print(xAccel);  print(" ");   print(yAccel);   print(" ");    print(zAccel);     print(" ");
    print(vRef);    print(" ");   print(xRate);    print(" ");    println(yRate);
    */
  }
}

// This class helps manage the arrays of data I need to keep around for graphing.
class cDataArray
{
  float[] m_data;
  int m_maxSize;
  int m_startIndex = 0;
  int m_endIndex = 0;
  int m_curSize;
  
  cDataArray(int maxSize)
  {
    m_maxSize = maxSize;
    m_data = new float[maxSize];
  }
  
  void addVal(float val)
  {
    
    if (g_enableFilter && (m_curSize != 0))
    {
      int indx;
      
      if (m_endIndex == 0)
        indx = m_maxSize-1;
      else
        indx = m_endIndex - 1;
      
      m_data[m_endIndex] = getVal(indx)*.5 + val*.5;
    }
    else
    {
      m_data[m_endIndex] = val;
    }
    
    m_endIndex = (m_endIndex+1)%m_maxSize;
    if (m_curSize == m_maxSize)
    {
      m_startIndex = (m_startIndex+1)%m_maxSize;
    }
    else
    {
      m_curSize++;
    }
  }
  
  float getVal(int index)
  {
    return m_data[(m_startIndex+index)%m_maxSize];
  }
  
  int getCurSize()
  {
    return m_curSize;
  }
  
  int getMaxSize()
  {
    return m_maxSize;
  }
}

// This class takes the data and helps graph it
class cGraph
{
  float m_gWidth, m_gHeight;
  float m_gLeft, m_gBottom, m_gRight, m_gTop;
  
  cGraph(float x, float y, float w, float h)
  {
    m_gWidth     = w;
    m_gHeight    = h;
    m_gLeft      = x;
    m_gBottom    = g_winH - y;
    m_gRight     = x + w;
    m_gTop       = g_winH - y - h;
  }
  
  void drawGraphBox()
  {
    stroke(0, 0, 0);
    rectMode(CORNERS);
    rect(m_gLeft, m_gBottom, m_gRight, m_gTop);
  }
  
  void drawLine(cDataArray data, float minRange, float maxRange)
  {
    float graphMultX = m_gWidth/data.getMaxSize();
    float graphMultY = m_gHeight/(maxRange-minRange);
    
    for(int i=0; i<data.getCurSize()-1; ++i)
    {
      float x0 = i*graphMultX+m_gLeft;
      float y0 = m_gBottom-((data.getVal(i)-minRange)*graphMultY);
      float x1 = (i+1)*graphMultX+m_gLeft;
      float y1 = m_gBottom-((data.getVal(i+1)-minRange)*graphMultY);
      line(x0, y0, x1, y1);
    }
  }
}
