#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795

// PCA9685 Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Define servo channels for each joint
const int joint1Servo = 0;
const int joint2Servo = 1;
const int joint3Servo = 2;
const int joint4Servo = 3;
const int joint5Servo = 4;
const int joint6Servo = 5;

// Define servo movement parameters (for MG 996R)
const int servoMin = 150; // Minimum servo position (approx. 0°)
const int servoMax = 600; // Maximum servo position (approx. 180°)
const int frameRate = 50; // Frames per second for movement (matches servo frequency)

// Arm dimensions (mm) - Assume these are correct for your arm
#define BASE_HGT 90
#define HUMERUS 100
#define ULNA 135
#define GRIPPER 200
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;
const double dl1 = 360.0/200.0/32.0/4.8;
const double dl2 = 360.0/200.0/32.0/4.0;
const double dl3 = 360.0/200.0/32.0/5.0;
const double dl4 = 360.0/200.0/32.0/2.8;
const double dl5 = 360.0/200.0/32.0/2.1;
const double dl6 = 360.0/200.0/32.0/1.0;
const double r1 = 47.0;
const double r2 = 110.0;
const double r3 = 26.0; 
const double d1 = 133.0;
const double d3 = 0.0;
const double d4 = 117.50;
const double d6 = 28.0;

// Current positions
double curPos1 = 0.0;
double curPos2 = 0.0;
double curPos3 = 0.0;
double curPos4 = 0.0;
double curPos5 = 0.0;
double curPos6 = 0.0;

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);
  
  // Initialize servo positions (all to 0° initially)
  pwm.setPWM(joint1Servo, 0, servoMin);
  pwm.setPWM(joint2Servo, 0, servoMin);
  pwm.setPWM(joint3Servo, 0, servoMin);
  pwm.setPWM(joint4Servo, 0, servoMin);
  pwm.setPWM(joint5Servo, 0, servoMin);
  pwm.setPWM(joint6Servo, 0, servoMin);
  
  delay(1000); // Wait for 1 second after initialization
}

void loop() {
  // Example target position in degrees for all joints
  float targetPosition[] = {90, 90, 90, 90, 90, 90};
  
  // Perform inverse kinematics to find joint positions
  float jointPositions[6];
  InverseK(targetPosition, jointPositions);
  
  // Move the arm to the calculated joint positions
  goTrajectory(jointPositions);
  
  delay(3000); // Wait for 1 second before next movement
}

void goTrajectory(float* Jf) {
  // Map joint positions to servo angles (considering MG 996R's 0° to 180° range)
  int servoAngles[] = {
    map(Jf, -180, 180, servoMin, servoMax),
    map(Jf[1], -180, 180, servoMin, servoMax),
    map(Jf[2], -180, 180, servoMin, servoMax),
    map(Jf[3], -180, 180, servoMin, servoMax),
    map(Jf[4], -180, 180, servoMin, servoMax),
    map(Jf[5], -180, 180, servoMin, servoMax)
  };

  // Set servo positions
  pwm.setPWM(joint1Servo, 0, servoAngles);
  pwm.setPWM(joint2Servo, 0, servoAngles[1]);
  pwm.setPWM(joint3Servo, 0, servoAngles[2]);
  pwm.setPWM(joint4Servo, 0, servoAngles[3]);
  pwm.setPWM(joint5Servo, 0, servoAngles[4]);
  pwm.setPWM(joint6Servo, 0, servoAngles[5]);

  // Delay for frame rate control (adjust as necessary for smooth movement)
  delay(3000 / frameRate);
}

// Inverse Kinematics function
void InverseK(float* Xik, float* Jik) {
  // inverse kinematics
  // input: Xik - pos value for the calculation of the inverse kinematics
  // output: Jfk - joints value for the calculation of the inversed kinematics
  
  // from deg to rad
  // Xik(4:6)=Xik(4:6)*pi/180;
  Xik[3]=Xik[3]*PI/180.0;
  Xik[4]=Xik[4]*PI/180.0;
  Xik[5]=Xik[5]*PI/180.0;
  
  // Denavit-Hartenberg matrix
  float theta[6]={0.0, -90.0, 0.0, 0.0, 0.0, 0.0}; // theta=[0; -90+0; 0; 0; 0; 0];
  float alfa[6]={-90.0, 0.0, -90.0, 90.0, -90.0, 0.0}; // alfa=[-90; 0; -90; 90; -90; 0];
  float r[6]={r1, r2, r3, 0.0, 0.0, 0.0}; // r=[47; 110; 26; 0; 0; 0];
  float d[6]={d1, 0.0, d3, d4, 0.0, d6}; // d=[133; 0; 7; 117.5; 0; 28];
  // from deg to rad
  MatrixScale(theta, 6, 1, PI/180.0); // theta=theta*pi/180;
  MatrixScale(alfa, 6, 1, PI/180.0); // alfa=alfa*pi/180;
  
  // work frame
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xwf=[0; 0; 0; 0; 0; 0];
  
  // tool frame
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xtf=[0; 0; 0; 0; 0; 0];
  
  // work frame transformation matrix
  float Twf[16];
  pos2tran(Xwf, Twf); // Twf=pos2tran(Xwf);
  
  // tool frame transformation matrix
  float Ttf[16];
  pos2tran(Xtf, Ttf); // Ttf=pos2tran(Xtf);
  
  // total transformation matrix
  float Twt[16];
  pos2tran(Xik, Twt); // Twt=pos2tran(Xik);
  
  // find T06
  float inTwf[16], inTtf[16], Tw6[16], T06[16];
  invtran(Twf, inTwf); // inTwf=invtran(Twf);
  invtran(Ttf, inTtf); // inTtf=invtran(Ttf);
  MatrixMultiply(Twt, inTtf, 4, 4, 4, Tw6); // Tw6=Twt*inTtf;
  MatrixMultiply(inTwf, Tw6, 4, 4, 4, T06); // T06=inTwf*Tw6;
  
  // positon of the spherical wrist
  float Xsw[3];
  // Xsw=T06(1:3,4)-d(6)*T06(1:3,3);
  Xsw[0]=T06[0*4 + 3]-d[5]*T06[0*4 + 2];
  Xsw[1]=T06[1*4 + 3]-d[5]*T06[1*4 + 2];
  Xsw[2]=T06[2*4 + 3]-d[5]*T06[2*4 + 2];
  
  // joints variable
  // Jik=zeros(6,1);
  // first joint
  Jik[0]=atan2(Xsw[1],Xsw[0])-atan2(d[2],sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])); // Jik(1)=atan2(Xsw(2),Xsw(1))-atan2(d(3),sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2));
  // second joint
  Jik[1]=PI/2.0
  -acos((r[1]*r[1]+(Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])-(r[2]*r[2]+d[3]*d[3]))/(2.0*r[1]*sqrt((Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))))
  -atan((Xsw[2]-d[0])/(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])); // Jik(2)=pi/2-acos((r(2)^2+(Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*sqrt((Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1)));
  // third joint
  Jik[2]=PI
  -acos((r[1]*r[1]+r[2]*r[2]+d[3]*d[3]-(Xsw[2]-d[0])*(Xsw[2]-d[0])-(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))/(2*r[1]*sqrt(r[2]*r[2]+d[3]*d[3])))
  -atan(d[3]/r[2]); // Jik(3)=pi-acos((r(2)^2+r(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*sqrt(r(3)^2+d(4)^2)))-atan(d(4)/r(3));
  // last three joints
  float T01[16], T12[16], T23[16], T02[16], T03[16], inT03[16], T36[16];
  DH1line(theta[0]+Jik[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1)+Jik(1),alfa(1),r(1),d(1));
  DH1line(theta[1]+Jik[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2)+Jik(2),alfa(2),r(2),d(2));
  DH1line(theta[2]+Jik[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3)+Jik(3),alfa(3),r(3),d(3));
  MatrixMultiply(T01, T12, 4, 4, 4, T02); // T02=T01*T12;
  MatrixMultiply(T02, T23, 4, 4, 4, T03); // T03=T02*T23;
  invtran(T03, inT03); // inT03=invtran(T03);
  MatrixMultiply(inT03, T06, 4, 4, 4, T36); // T36=inT03*T06;
  // forth joint
  Jik[3]=atan2(-T36[1*4+2], -T36[0*4+2]); // Jik(4)=atan2(-T36(2,3),-T36(1,3));
  // fifth joint
  Jik[4]=atan2(sqrt(T36[0*4+2]*T36[0*4+2]+T36[1*4+2]*T36[1*4+2]), T36[2*4+2]); // Jik(5)=atan2(sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3));
  // sixth joints
  Jik[5]=atan2(-T36[2*4+1], T36[2*4+0]); // Jik(6)=atan2(-T36(3,2),T36(3,1));
  // rad to deg
  MatrixScale(Jik, 6, 1, 180.0/PI); // Jik=Jik/pi*180;
}


// Forward Kinematics function
void ForwardK(float* Jfk, float* Xfk) {
  // forward kinematics
  // input: Jfk - joints value for the calculation of the forward kinematics
  // output: Xfk - pos value for the calculation of the forward kinematics
  
  // Denavit-Hartenberg matrix
  float theTemp[6]={0.0, -90.0, 0.0, 0.0, 0.0, 0.0};
  float theta[6];
  MatrixAdd(theTemp, Jfk, 6, 1, theta); // theta=[Jfk(1); -90+Jfk(2); Jfk(3); Jfk(4); Jfk(5); Jfk(6)];
  float alfa[6]={-90.0, 0.0, -90.0, 90.0, -90.0, 0.0}; // alfa=[-90; 0; -90; 90; -90; 0];
  float r[6]={r1, r2, r3, 0.0, 0.0, 0.0}; // r=[47; 110; 26; 0; 0; 0];
  float d[6]={d1, 0.0, d3, d4, 0.0, d6}; // d=[133; 0; 7; 117.5; 0; 28];
  // from deg to rad
  MatrixScale(theta, 6, 1, PI/180.0); // theta=theta*pi/180;
  MatrixScale(alfa, 6, 1, PI/180.0); // alfa=alfa*pi/180;
  
  // work frame
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xwf=[0; 0; 0; 0; 0; 0];
  
  // tool frame
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xtf=[0; 0; 0; 0; 0; 0];
  
  // work frame transformation matrix
  float Twf[16];
  pos2tran(Xwf, Twf); // Twf=pos2tran(Xwf);
  
  // tool frame transformation matrix
  float Ttf[16];
  pos2tran(Xtf, Ttf); // Ttf=pos2tran(Xtf);
  
  // DH homogeneous transformation matrix
  float T01[16], T12[16], T23[16], T34[16], T45[16], T56[16];
  DH1line(theta[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1),alfa(1),r(1),d(1));
  DH1line(theta[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2),alfa(2),r(2),d(2));
  DH1line(theta[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3),alfa(3),r(3),d(3));
  DH1line(theta[3], alfa[3], r[3], d[3], T34); // T34=DH1line(theta(4),alfa(4),r(4),d(4));
  DH1line(theta[4], alfa[4], r[4], d[4], T45); // T45=DH1line(theta(5),alfa(5),r(5),d(5));
  DH1line(theta[5], alfa[5], r[5], d[5], T56); // T56=DH1line(theta(6),alfa(6),r(6),d(6));

  float Tw1[16], Tw2[16], Tw3[16], Tw4[16], Tw5[16], Tw6[16], Twt[16];
  MatrixMultiply(Twf, T01, 4, 4, 4, Tw1); // Tw1=Twf*T01;
  MatrixMultiply(Tw1, T12, 4, 4, 4, Tw2); // Tw2=Tw1*T12;
  MatrixMultiply(Tw2, T23, 4, 4, 4, Tw3); // Tw3=Tw2*T23;
  MatrixMultiply(Tw3, T34, 4, 4, 4, Tw4); // Tw4=Tw3*T34;
  MatrixMultiply(Tw4, T45, 4, 4, 4, Tw5); // Tw5=Tw4*T45;
  MatrixMultiply(Tw5, T56, 4, 4, 4, Tw6); // Tw6=Tw5*T56;
  MatrixMultiply(Tw6, Ttf, 4, 4, 4, Twt); // Twt=Tw6*Ttf;
  
  // calculate pos from transformation matrix
  tran2pos(Twt, Xfk); // Xfk=tran2pos(Twt);
  // Xfk(4:6)=Xfk(4:6)/pi*180;
  Xfk[3]=Xfk[3]/PI*180.0;
  Xfk[4]=Xfk[4]/PI*180.0;
  Xfk[5]=Xfk[5]/PI*180.0;
}

// Other utility functions 
void invtran(float* Titi, float* Titf)
{
  // finding the inverse of the homogeneous transformation matrix
  // first row
  Titf[0*4 + 0] = Titi[0*4 + 0];
  Titf[0*4 + 1] = Titi[1*4 + 0];
  Titf[0*4 + 2] = Titi[2*4 + 0];
  Titf[0*4 + 3] = -Titi[0*4 + 0]*Titi[0*4 + 3]-Titi[1*4 + 0]*Titi[1*4 + 3]-Titi[2*4 + 0]*Titi[2*4 + 3];
  // second row
  Titf[1*4 + 0] = Titi[0*4 + 1];
  Titf[1*4 + 1] = Titi[1*4 + 1];
  Titf[1*4 + 2] = Titi[2*4 + 1];
  Titf[1*4 + 3] = -Titi[0*4 + 1]*Titi[0*4 + 3]-Titi[1*4 + 1]*Titi[1*4 + 3]-Titi[2*4 + 1]*Titi[2*4 + 3];
  // third row
  Titf[2*4 + 0] = Titi[0*4 + 2];
  Titf[2*4 + 1] = Titi[1*4 + 2];
  Titf[2*4 + 2] = Titi[2*4 + 2];
  Titf[2*4 + 3] = -Titi[0*4 + 2]*Titi[0*4 + 3]-Titi[1*4 + 2]*Titi[1*4 + 3]-Titi[2*4 + 2]*Titi[2*4 + 3];
  // forth row
  Titf[3*4 + 0] = 0.0;
  Titf[3*4 + 1] = 0.0;
  Titf[3*4 + 2] = 0.0;
  Titf[3*4 + 3] = 1.0;
}

void tran2pos(float* Ttp, float* Xtp)
{
  // pos from homogeneous transformation matrix
  Xtp[0] = Ttp[0*4 + 3];
  Xtp[1] = Ttp[1*4 + 3];
  Xtp[2] = Ttp[2*4 + 3];
  Xtp[4] = atan2(sqrt(Ttp[2*4 + 0]*Ttp[2*4 + 0] + Ttp[2*4 + 1]*Ttp[2*4 + 1]),Ttp[2*4 + 2]);
  Xtp[3] = atan2(Ttp[1*4 + 2]/sin(Xtp[4]),Ttp[0*4 + 2]/sin(Xtp[4]));
  Xtp[5] = atan2(Ttp[2*4 + 1]/sin(Xtp[4]),-Ttp[2*4 + 0]/sin(Xtp[4]));
}


void pos2tran(float* Xpt, float* Tpt)
{
  // pos to homogeneous transformation matrix
  // first row
  Tpt[0*4 + 0] = cos(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])-sin(Xpt[3])*sin(Xpt[5]);
  Tpt[0*4 + 1] = -cos(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])-sin(Xpt[3])*cos(Xpt[5]);
  Tpt[0*4 + 2] = cos(Xpt[3])*sin(Xpt[4]);
  Tpt[0*4 + 3] = Xpt[0];
  // second row
  Tpt[1*4 + 0] = sin(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])+cos(Xpt[3])*sin(Xpt[5]);
  Tpt[1*4 + 1] = -sin(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])+cos(Xpt[3])*cos(Xpt[5]);
  Tpt[1*4 + 2] = sin(Xpt[3])*sin(Xpt[4]);
  Tpt[1*4 + 3] = Xpt[1];
  // third row
  Tpt[2*4 + 0] = -sin(Xpt[4])*cos(Xpt[5]);
  Tpt[2*4 + 1] = sin(Xpt[4])*sin(Xpt[5]);
  Tpt[2*4 + 2] = cos(Xpt[4]);
  Tpt[2*4 + 3] = Xpt[2];
  // forth row
  Tpt[3*4 + 0] = 0.0;
  Tpt[3*4 + 1] = 0.0;
  Tpt[3*4 + 2] = 0.0;
  Tpt[3*4 + 3] = 1.0;
}


void DH1line(float thetadh, float alfadh, float rdh, float ddh, float* Tdh)
{
  // creats Denavit-Hartenberg homogeneous transformation matrix
  // first row
  Tdh[0*4 + 0] = cos(thetadh);
  Tdh[0*4 + 1] = -sin(thetadh)*cos(alfadh);
  Tdh[0*4 + 2] = sin(thetadh)*sin(alfadh);
  Tdh[0*4 + 3] = rdh*cos(thetadh);
  // second row
  Tdh[1*4 + 0] = sin(thetadh);
  Tdh[1*4 + 1] = cos(thetadh)*cos(alfadh);
  Tdh[1*4 + 2] = -cos(thetadh)*sin(alfadh);
  Tdh[1*4 + 3] = rdh*sin(thetadh);
  // third row
  Tdh[2*4 + 0] = 0.0;
  Tdh[2*4 + 1] = sin(alfadh);
  Tdh[2*4 + 2] = cos(alfadh);
  Tdh[2*4 + 3] = ddh;
  // forth row
  Tdh[3*4 + 0] = 0.0;
  Tdh[3*4 + 1] = 0.0;
  Tdh[3*4 + 2] = 0.0;
  Tdh[3*4 + 3] = 1.0;
}


void MatrixPrint(float* A, int m, int n, String label)
{
  // A = input matrix (m x n)
  int i, j;
  Serial.println();
  Serial.println(label);
  for (i = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
    {
      Serial.print(A[n * i + j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void MatrixCopy(float* A, int n, int m, float* B)
{
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
    {
      B[n * i + j] = A[n * i + j];
    }
}

//Matrix Multiplication Routine
// C = A*B
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)
  int i, j, k;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
    {
      C[n * i + j] = 0;
      for (k = 0; k < p; k++)
        C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
    }
}


//Matrix Addition Routine
void MatrixAdd(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A+B (m x n)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] + B[n * i + j];
}


//Matrix Subtraction Routine
void MatrixSubtract(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A-B (m x n)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] - B[n * i + j];
}


//Matrix Transpose Routine
void MatrixTranspose(float* A, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // m = number of rows in A
  // n = number of columns in A
  // C = output matrix = the transpose of A (n x m)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[m * j + i] = A[n * i + j];
}

void MatrixScale(float* A, int m, int n, float k)
{
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      A[n * i + j] = A[n * i + j] * k;
}