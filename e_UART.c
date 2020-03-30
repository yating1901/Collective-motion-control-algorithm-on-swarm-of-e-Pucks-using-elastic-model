 #include "p30F6014A.h"                          //调用头文件
#include "include.h"
#include <string.h>
#include <math.h>
#include "e_agenda.h"
#include "e_init_port.h"
#include "e_motors.h"
CAR_POS car[car_num];
extern int ID;
extern  float angle_media;
extern CAR_POS Robot_position[16];
extern CAR_POS Robot_record[16];
extern unsigned int original_angel;
int car_ID;
int state_machine=1;
int counter=0;
int data_count=0;
int communication_state;
int formation;
char theata_flag=0;
double D_value=0.0;
#define TURN_SPEED 1000
#define STEPS_FOR_2PI 1300.
int del_x=0;
int del_y=0;
int del_f=0;
unsigned char rcvdat;
unsigned char RX_Buffer[100];
unsigned char *buf_ptr;
unsigned char *end_ptr;
int left_wheel_speed =0;    //200;
int right_wheel_speed =0;   //200;
int start_flag=0;
static int Neighbour_list[6][3] =
{ 
	{10,1,11},  //A0:B1K10L11
    {0,2,11},  //B1:A0C2L11
    {1,3,11}, //C2:B1D3L11
    {2,5,11}, //D3:C2F5L11
    {3,10,11}, //F5:D3K10L11
    {5,0,11} //K10:F5A0L11
};
//static float V0[7] ={1.6346,2.4319,0.9921,0.6495,0.9921,1.6346,2.3419};

void Delay_xmS(unsigned int i)                  //延时程序
{
	unsigned int j;
	for(;i>0;i--)
	{
		Nop();
		for(j=0;j<255;j++)
		{
			Nop();
			ClrWdt(); 
		}
	}	
}

void UART2_Init(void)
{
	U2MODEbits.STSEL = 0;            // 1个停止位
	U2MODEbits.PDSEL = 0;            // N , 8, 1
	U2MODEbits.ABAUD = 0;            // 自动波特率禁止
//	U1MODEbits.RTSMD = 1;            // 引脚处于单工模式

	U2BRG = U1BRG = (int)((FCY/ BAUDRATE)/16)-1;                      // UxBRG = ((FCY/ 目标波特率)/16)-1

	U2STAbits.UTXISEL = 1;           // 发送一个字符后中断
	                                                 
	U2STAbits.URXISEL1 = 0;          // 接收一个字符后中断
	U2STAbits.URXISEL0 = 0;
	
	IEC1bits.U2RXIE = 1;             // 允许接收中断
//	IEC1bits.U2TXIE = 1;             // 允许发送中断
	
	U2MODEbits.UARTEN = 1;           // 允许发送
	U2STAbits.UTXEN   = 1;           // 使能串口2
}

void __attribute__((__interrupt__,no_auto_psv))_U2RXInterrupt(void)
{
	IEC1bits.U2RXIE = 0;
	rcvdat = U2RXREG;
    
    RX_Buffer[counter++] = rcvdat;

    if (rcvdat=='G'&&state_machine==1){
    data_count=counter-1; //记录数据开始位置
    buf_ptr=&RX_Buffer[data_count];
    state_machine=0;
    }
    if (rcvdat=='$'){

          end_ptr=&RX_Buffer[counter-1];

          if((end_ptr-buf_ptr)==8){
             Reverse_data();
             state_machine=1;
              if(counter>81)counter=0; 
          }           
           
    }
   if (counter>=95) counter=0;

	IFS1bits.U2RXIF = 0;
	IEC1bits.U2RXIE = 1;
}	


void speed_control(int num_of_data)//char Robot_Tag
{
    double L_ij=0.15; //meter 直径0.075m
    double K_ij=0.015;//0.015;0.005;0.01;0.015;0.005;0.01;0.005；0.015;     
    double Alpha=20.0; //15；10 
    double Beta=133.3;   //533.2-80;400-80;266.6-40;200-30 66.6-10，133.3-20
    double V_0=115.0;  //76-0.01;38-0.005;115-0.015
    double N_x=0.0;
    double N_y=0.0;
    double F_x=0.0;
    double F_y=0.0;
    double delta_x=0.0;
    double delta_y=0.0;
    double Nx_orthogoral=0.0;
    double Ny_orthogoral=0.0; 
    double robot_theata=0;
    double deta_position=0.0;
    double robot_width=0.0535; //轮距,meter
    double wheel_width=0.0412;
    double delta_angle=0.0;
    double inter_rate=0.00050;//1 pixels to 0.052 meter
    double inverse_rate=2000.0;//1 meter to 1923 pixels
    int delta_xx=0;
    int delta_yy=0;
    int p_ID=0; 
    int is_neighbour=0;           
    unsigned int delta_distance=0.0;
    int k=0;
    V_0=V_0*0.00013;  

    robot_theata=(double)Robot_position[ID].Theta.f; //自己的角度
    robot_theata=(360-robot_theata-90);
    if(robot_theata<0) robot_theata=robot_theata+360;
  
    for(p_ID=0;p_ID<16;p_ID++)
   {   
       if(ID==11) is_neighbour=1;     //处在中心的L不需要找neignbour;
       else
           { is_neighbour=find_neighbour(p_ID);}
       
       if(Robot_position[p_ID].X.f!=0.0 && Robot_position[p_ID].Y.f!=0.0 && p_ID!=ID && is_neighbour!=0) 
        {              
          delta_xx=Robot_position[p_ID].X.f-Robot_position[ID].X.f;
          delta_yy=Robot_position[p_ID].Y.f-Robot_position[ID].Y.f;
          delta_x=(double)delta_xx; //
          delta_y=(double)delta_yy;//像素距离
          delta_distance=(int)sqrt(delta_x*delta_x+delta_y*delta_y);  // 像素距离
          N_x=cos(robot_theata*PI/180);
          N_y=sin(robot_theata*PI/180);
          Nx_orthogoral=cos((robot_theata+90)*PI/180);
          Ny_orthogoral=sin((robot_theata+90)*PI/180);
          F_x=F_x-K_ij/L_ij *(L_ij-delta_distance*inter_rate)* delta_x/delta_distance;         
          F_y=F_y-K_ij/L_ij *(L_ij-delta_distance*inter_rate)* delta_y/delta_distance;        
        }
is_neighbour=0;
}   //for循环结束   
          deta_position=Alpha*(F_x*N_x+F_y*N_y);  //x_dot  int*double to double
          delta_angle=Beta*(F_x*Nx_orthogoral+F_y*Ny_orthogoral); //theata_dot 注：弧度,double
          left_wheel_speed=(int)((V_0+deta_position+robot_width*delta_angle/2)*1000/0.13); 
          right_wheel_speed=(int)((V_0+deta_position-robot_width*delta_angle/2)*1000/0.13); //double to int
          //walking strightly
          if(left_wheel_speed > 400) 
             left_wheel_speed = 400;
		        else if(left_wheel_speed <-400)  
                   left_wheel_speed = -400;
          	
           if(right_wheel_speed > 400) 
              right_wheel_speed = 400;
	            else if(right_wheel_speed <-400)  
                   right_wheel_speed = -400;         
 for(p_ID=0;p_ID<16;p_ID++){
              Robot_position[p_ID].Theta.f=0;//注意这里或许并不能直接清除数据;需要重新设置中间数组
              Robot_position[p_ID].X.f=0;
              Robot_position[p_ID].Y.f=0;
  } 
F_x=0;
F_y=0;
}
void turn_to_direction(float direction)
{
	int end_turn;
		
		if (direction < 0)     // turn clockwise
		{
			e_set_steps_left(0);
			// calculate how many steps the robot needs to turn
			end_turn = (int)(STEPS_FOR_2PI*(-direction/(2*PI)));   

			e_set_speed_left(TURN_SPEED);  // motor speed in steps/s
			e_set_speed_right(-TURN_SPEED);  // motor speed in steps/s
			
			while(e_get_steps_left() < end_turn);   // turn until done 
		}
		else 					// turn counterclockwise
		{
			e_set_steps_right(0);
			// calculate how many steps the robot needs to turn
			end_turn = (int)(STEPS_FOR_2PI*((direction)/(2*PI)));

			e_set_speed_left(-TURN_SPEED);  // motor speed in steps/s
			e_set_speed_right(TURN_SPEED);  // motor speed in steps/s
			
			while(e_get_steps_right() < end_turn);   // turn until done
		}
}

void Reverse_data(){
      unsigned char *p;
      char Tag=0;
      p=buf_ptr;
      int i=0;
if(*p++=='G'){
   
      Tag=*p++;
if(Tag>=0&&Tag<=15){
      Robot_position[Tag].X.c[0]=*p++;
      Robot_position[Tag].X.c[1]=*p++;
      Robot_position[Tag].Y.c[0]=*p++;
      Robot_position[Tag].Y.c[1]=*p++;
      Robot_position[Tag].Theta.c[0]=*p++;
      Robot_position[Tag].Theta.c[1]=*p++;
}
/*
if(*p=='$'){
       if(Tag==ID) theata_flag=1; 
}
*/
}
}

void __attribute__((__interrupt__,no_auto_psv))_U2TXInterrupt(void)
{
	IFS1bits.U2TXIF = 0;
}

int find_neighbour(int Neighbour_ID)
{
   int robot_line=0;
   int neigh_row=0;
   int Is_neighbour=0;
   int neighbour_differ=0;
   
   robot_line=ID;
   if(ID==5) robot_line=ID-1;
   if(ID==10) robot_line=ID-5;
  // if(ID==11) robot_line=ID-5; 
   for(neigh_row=0;neigh_row<3;neigh_row++){
     neighbour_differ=Neighbour_list[robot_line][neigh_row]-Neighbour_ID;
       if (neighbour_differ==0)  Is_neighbour=1;
   }

return Is_neighbour;

}


