#include "p30F6014A.h"
#include "include.h"
#include <math.h> 	

int ID;	
extern CAR_POS car[car_num];
CAR_POS car_goal[car_num];
CAR_POS car_init[car_num];
Vector Fi,Fo,Ff,F;
Vector F_id[car_num];
double *dis_ij;	
double dis[4];
extern CAR_POS Robot_position[16]; //用来储存每次接受电脑数据自己的角度
extern unsigned int original_angel;
int get_car_ID(void) 
{
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}


void car_Init(void)
{   
int i=0;
   while(1)       //发送初始位置信息
    {
   if(Robot_position[ID].X.f!=0&& Robot_position[ID].Y.f!=0)
    {
//     original_angel =Robot_position[ID].Theta.f;
     Delay_xmS(2000);
     break;
    }

  }
}



double *distance(CAR_POS from, CAR_POS to)
{

	double delta_x;
	double delta_y;
	double theta;
	double val;
	double *dist;
	dist = &dis[0];
	delta_x = to.X.f - from.X.f;
	delta_y = to.Y.f - from.Y.f;
	val = sqrt(delta_x*delta_x + delta_y*delta_y);
	dis[0] = delta_x;
	dis[1] = delta_y;
	dis[2] = val;
	return dist;
}


void Force_f(void)
{
	double *dis_g;
	dis_g = distance(car[ID],car_goal[ID]);
	Ff.x = dis_g[0];
	Ff.y = dis_g[1];

}

void Force_ij(void)
{
	int i;

	double dij;
	double sumx,sumy;
	for(i = 0; i<3; i++)
	{
		dis_ij = distance(car[ID],car[i]);
		dij = dis_ij[2];       //两个机器人之间的距离
		if(dij < 0.05)
			dij = 0.05;

		if(dij < d0 && i != ID)   //d0=0.3;
		{
			F_id[i].val = (1/dij - 1/d0);     
			
			F_id[i].x = F_id[i].val*dis_ij[0]/dij;
			F_id[i].y = F_id[i].val*dis_ij[1]/dij;
		}	
		else
		{
			F_id[i].val = 0;
			F_id[i].x = 0;
			F_id[i].y = 0;
		}
	}
	
	Fi.x = F_id[0].x + F_id[1].x +F_id[2].x;
	Fi.y = F_id[0].y + F_id[1].y +F_id[2].y;

}

void Joint_Force(void)
{
	F.x = 0.6*Ff.x - 0.2*Fi.x;    //???
	F.y = 0.6*Ff.y - 0.2*Fi.y;
	F.val = sqrt(F.x*F.x + F.y*F.y);

	if (F.val < 0.0001)
	{
		F.theta = 0;
	}
	else
	{
		F.theta = asin(F.y/F.val);
	}
	if(F.x < 0 && F.y > 0)
		F.theta = PI - F.theta;
	else if(F.x < 0 && F.y < 0)
		F.theta = -PI - F.theta;
	F.x = Kc*F.x;
	F.y = Kc*F.y;                //？如何确定Kc=2000;
	F.val = Kc*F.val;
}

void contrl()
{
	int u1,u2;
	double delta_theta;
	u1 = F.val;
	delta_theta = F.theta - car[ID].Theta.f;
	if(delta_theta > PI)
		delta_theta -= 2*PI;
	else if(delta_theta < -PI)
		delta_theta += 2*PI;
	u2 = Komega*delta_theta;       ////Komega=800
	if(u1 > 30)
		u1 = 500;
	else if(u1 <-30)
		u1 = -500;
	if(u2 > 500)
		u2 = 500;
	else if(u2 <-600)
		u2 = -500;
	if(abs(u2) > 10)
		u1 = 0;
	e_set_speed(u1,u2);
}
