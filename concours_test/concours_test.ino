#include <avr/io.h>
#include "robopoly.h"
#include "LinearCamera.h"
#include <avr/interrupt.h>

#define LED_PORT PORTC
#define LED_PIN 2

#define IR_LINE_FRONT 0
#define IR_LINE_LEFT 1
#define IR_LINE_RIGHT 2
#define IR_RIGHT 6
#define IR_LEFT 5
#define IR_BLACK_CHECK 4
#define IR_BOT 3
#define IR_BACK 7
#define INTERRUPT_PIN 0


#define SERVO 0
#define UP 8
#define DOWN 82

#define IR_WHITE_BACK 15
#define IR_WHITE 20
#define IR_BLACK 60
#define IR_OBSTACLE 150
#define IR_BLACK_VERIFY 170
#define IR_BACK_LIMIT 160

#define CAM_INTEGRATE 2900
#define TURN_SPEED 25
#define SPEED 70
#define FOLLOW_TURN_SPEED 20
#define FOLLOW_SPEED 40
#define LINE_TURN_LIMIT 400
#define RESSOURCE_WAIT 500
#define CATCH_SPEED 30
#define LIGHT_CHASE_CORRECTOR 3


// Note for Lcam: if lcam_getpic() gives > 12, light is on the left side of the cam, else on the right side.
void turn(int angle); // Angle is positive to the left, in degrees.
void robot_move(int distance); // move in straight line. Distance given in cm, approximatively.

void robot_begin(void);
void find_light(void);
int chase_light(void);
void dodge_obstacle(void);
int catch_ressource(void);
int get_back_home(void);

unsigned char cam;
unsigned char ir_black_check;
unsigned char ir_bot;
unsigned char ir_left;
unsigned char ir_right;
unsigned char ir_line_left;
unsigned char ir_line_right;
unsigned char ir_line_front;
unsigned char ir_back;
unsigned char interrupted = 0;

int main()
{

    lcam_setup();
 //   noInterrupts();
 // attachInterrupt(0, interrupt, RISING);
   // attachInterrupt(1, interrupt_capteur, FALLING);
    //noInterrupts();
   robot_begin();


    while(1)
    {
        if(chase_light())
           if(catch_ressource())
             get_back_home();
      
    }

}

void robot_begin(void)
{
  ir_bot = analogReadPortA(IR_BOT);
  ir_left = analogReadPortA(IR_LEFT);
  ir_right = analogReadPortA(IR_RIGHT);
  
  while(ir_bot < IR_WHITE || ir_left < IR_WHITE || ir_right < IR_WHITE)
  {
      waitms(10);
      ir_bot = analogReadPortA(IR_BOT);
      ir_left = analogReadPortA(IR_LEFT);
      ir_right = analogReadPortA(IR_RIGHT);
  }
  
  
  setSpeed(SPEED, SPEED-5);
  waitms(2000);
  setServo(SERVO, UP);
  waitms(10);
  setSpeed(-TURN_SPEED, TURN_SPEED);
  waitms(500);
}



int chase_light(void)
{
    ir_back = analogReadPortA(IR_BACK);
    
    lcam_integrate(CAM_INTEGRATE);
    lcam_read();
    cam = lcam_getpic();
    int speed = cam - 12;
    unsigned int k = 0;
    unsigned char move_away = 0;

    // a gauche: cam = 12, on veut la roue gauche moins rapide que la droite.
    // a droite: cam = -12, on veut la roue droite moins rapide que la gauche.
    // Donc comme speed = 12 a gauche, on ajoute a gauche, on soustrait a droite.
    while(ir_back > IR_BACK_LIMIT)
    {
      setServo(SERVO, UP);
        if(cam != 0)
        {
          setSpeed(-SPEED + speed, -SPEED - speed);
        }
        else
        {
          setSpeed(-TURN_SPEED, TURN_SPEED);
          k++;
        }
          
        waitms(30);        
        if(k > 200)
        {
          k = 0;
          move_away = 1;
        }
        
        while(move_away)
        {

          ir_left = analogReadPortA(IR_LEFT);
          ir_right = analogReadPortA(IR_RIGHT);
          ir_bot = analogReadPortA(IR_BOT);
          while(ir_left < IR_OBSTACLE || ir_right < IR_OBSTACLE || ir_bot < IR_OBSTACLE)
          {
            if(ir_bot < IR_OBSTACLE)
            {
              setSpeed(-SPEED, -SPEED);
              waitms(300);
              setSpeed(-TURN_SPEED, TURN_SPEED);
              waitms(200);
            }
            else if(ir_left < IR_OBSTACLE)
            {
              setSpeed(-TURN_SPEED, TURN_SPEED);
              waitms(100);
            }
            else
            {
              setSpeed(TURN_SPEED, -TURN_SPEED);
              waitms(100);
            }
          }

          setSpeed(SPEED, SPEED);
          waitms(300);

          lcam_integrate(CAM_INTEGRATE);
          lcam_read();    
          cam = lcam_getpic();
          if(cam)
          {
            move_away = 0;
          }
        }
          
        lcam_integrate(CAM_INTEGRATE);
        lcam_read();    
        cam = lcam_getpic();
        speed = LIGHT_CHASE_CORRECTOR*(12 - cam);
        
        ir_back = analogReadPortA(IR_BACK);
    }
    
    setSpeed(0,0);       
    waitms(10);    
    return 1;
}


int catch_ressource(void)
{
  setServo(SERVO, DOWN);
  waitms(100);  
  ir_back = analogReadPortA(IR_BACK);
  waitms(20);
  if(ir_back < IR_WHITE_BACK)
  {    
    setSpeed(SPEED,SPEED);
    waitms(500);
    ir_back = analogReadPortA(IR_BACK);
    if(ir_back < IR_WHITE_BACK)
      return 1;
    return 0;
  }
  setSpeed(SPEED, SPEED);
  waitms(1000);
  return 0;
}
  

int get_back_home()
{
  ir_line_left = analogReadPortA(IR_LINE_LEFT);
  ir_line_right = analogReadPortA(IR_LINE_RIGHT);
  ir_line_front = analogReadPortA(IR_LINE_FRONT);
  
  while(ir_line_front < IR_WHITE && ir_line_left < IR_WHITE && ir_line_right < IR_WHITE)
  {
    ir_line_left = analogReadPortA(IR_LINE_LEFT);
    ir_line_right = analogReadPortA(IR_LINE_RIGHT);
    ir_line_front = analogReadPortA(IR_LINE_FRONT);
    ir_left = analogReadPortA(IR_LEFT);
    ir_right = analogReadPortA(IR_RIGHT);
    ir_bot = analogReadPortA(IR_BOT);

    
    while(ir_left < IR_OBSTACLE || ir_right < IR_OBSTACLE || ir_bot < IR_OBSTACLE)
    {
      if(ir_bot < IR_OBSTACLE)
      {
        setSpeed(-SPEED, -SPEED);
        waitms(400);
        setSpeed(SPEED, -SPEED);
        waitms(400);
      }
      else if(ir_left < IR_OBSTACLE)
      {
        setSpeed(SPEED, -SPEED);
      }
      else
      {
        setSpeed(-SPEED, SPEED);
      }
      
    waitms(40);
    ir_left = analogReadPortA(IR_LEFT);
    ir_right = analogReadPortA(IR_RIGHT);
    ir_bot = analogReadPortA(IR_BOT);      
    } 
    
    setSpeed(SPEED, SPEED);  
    waitms(10);
    
  }
  setSpeed(0,0);
  waitms(200);
  setSpeed(-SPEED, -SPEED);
  waitms(100);
  setSpeed(0,0);
  waitms(20);

    ir_line_left = analogReadPortA(IR_LINE_LEFT);
    ir_line_right = analogReadPortA(IR_LINE_RIGHT);
    ir_line_front = analogReadPortA(IR_LINE_FRONT);
    ir_black_check = analogReadPortA(IR_BLACK_CHECK);
    ir_bot = analogReadPortA(IR_BOT);
    ir_left = analogReadPortA(IR_LEFT);
    ir_right = analogReadPortA(IR_RIGHT);
  while(ir_line_left > IR_BLACK && ir_line_right > IR_BLACK)
  {
    if(ir_line_left > IR_BLACK)
      setSpeed(-FOLLOW_TURN_SPEED, FOLLOW_TURN_SPEED);
    else
      setSpeed(FOLLOW_TURN_SPEED, FOLLOW_TURN_SPEED);
    
    ir_line_left = analogReadPortA(IR_LINE_LEFT);
    ir_line_right = analogReadPortA(IR_LINE_RIGHT);
    ir_line_front = analogReadPortA(IR_LINE_FRONT);
    ir_black_check = analogReadPortA(IR_BLACK_CHECK);
    ir_bot = analogReadPortA(IR_BOT);
    ir_left = analogReadPortA(IR_LEFT);
    ir_right = analogReadPortA(IR_RIGHT);
    waitms(20);
  }
  
  while(1)
  {
    if(ir_bot < IR_OBSTACLE)
      setSpeed(0,0);
	else if(ir_line_left > IR_BLACK && ir_line_right > IR_BLACK)
	{
		setSpeed(-FOLLOW_TURN_SPEED, FOLLOW_TURN_SPEED);
		waitms(250);
              setSpeed(0,0);
              waitms(20);
    }
    else if(ir_line_left > IR_BLACK)
    {
      setSpeed(-FOLLOW_TURN_SPEED,FOLLOW_TURN_SPEED);
    }
    else if(ir_line_right > IR_BLACK)
    {
      setSpeed(FOLLOW_TURN_SPEED,-FOLLOW_TURN_SPEED);
    }
    else 
      setSpeed(FOLLOW_SPEED,FOLLOW_SPEED);
        
    waitms(30);
    ir_line_left = analogReadPortA(IR_LINE_LEFT);
    ir_line_right = analogReadPortA(IR_LINE_RIGHT);
    ir_line_front = analogReadPortA(IR_LINE_FRONT);
    ir_black_check = analogReadPortA(IR_BLACK_CHECK);
    ir_bot = analogReadPortA(IR_BOT);

   /* if(ir_black_check < IR_BLACK_VERIFY && ir_line_front < IR_WHITE && ir_line_right < IR_WHITE && ir_line_left < IR_WHITE)
    {
      if(get_back_home())
        return 1;
      return 0;
    }*/
    if(ir_black_check > IR_BLACK_VERIFY && ir_line_front > IR_BLACK && (ir_line_left > IR_BLACK || ir_line_right > IR_BLACK))   
    {
      setSpeed(SPEED,SPEED);
      waitms(200);
      turn(180);
      setServo(SERVO,UP);
      while(ir_back < IR_BACK_LIMIT)
      {
        setSpeed(SPEED, SPEED);
        waitms(40);
        ir_back = analogReadPortA(IR_BACK);
      }
      unsigned char counter = 0;
      while(ir_bot> IR_OBSTACLE && ir_left > IR_OBSTACLE && ir_right > IR_OBSTACLE && counter < 180)
      {
        setSpeed(SPEED,SPEED);
        waitms(10);
        ir_bot = analogReadPortA(IR_BOT);
        ir_left = analogReadPortA(IR_LEFT);
        ir_right = analogReadPortA(IR_RIGHT);
        counter++;
      }

      return 1;
    }
 
  }

    return 0;
}


void interrupt(void)
{
  setSpeed(-SPEED,-SPEED);
  waitms(600);
  setSpeed(-TURN_SPEED, TURN_SPEED);
  waitms(500);
  return;
}

void interrupt_capteur(void)
{
  setSpeed(-SPEED, -SPEED);
  waitms(600);
  setSpeed(TURN_SPEED, -TURN_SPEED);
  waitms(500);
  return;
}

void turn(int angle)
{
    if (angle > 0)
    {
        setSpeed(-22,22);
        waitms(angle*7);
    }
    else
    {
        setSpeed(22,-22);
        waitms(-angle*7);
    }


    setSpeed(0,0);
    waitms(50);
}

void robot_move(int distance)
{
    if(distance > 0)
    {
        setSpeed(30,30);
        waitms(distance*49);
    }
    else
    {
        setSpeed(-30,-30);
        waitms(-distance*49);
    }

    setSpeed(0,0);
    waitms(50);
}



