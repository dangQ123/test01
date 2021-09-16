#include"DataStruct.h"
#include "math.h"

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <windows.h>
#include<QDebug>

#define cout qDebug()
#define PI 3.1415926

double deltaTime=0;
void startSituation(KeyStruct *key);

//角度化为弧度
double angleToRadian(double angle)
{
    return angle * PI / 180;
}

//向量
class Vector
{
public:
    double x, y;
    Vector()
    {
        x = 0;
        y = 0;
    }
    Vector(double _x, double _y)
    {
        x = _x;
        y = _y;
    }
    Vector(double angle)
    {
        double radian = angleToRadian(angle);
        x = -sin(radian);
        y = -cos(radian);
    }

    double length()
    {
        return sqrt(x * x + y * y);
    }
    double operator*(Vector& a)
    {
        return this->x * a.x + this->y * a.y;
    }

};

//点
class Point
{
public:
    double x, y;
    Point()
    {
        this->x = 0;
        this->y = 0;
    }
    Point(double _x, double _y)
    {
        x = _x;
        y = _y;
    }
    Vector operator-(Point a)
    {
        Vector temp(a.x - this->x, a.y - this->y);
        return temp;
    }
    void showSelf()
    {
        cout << "x=" << x << " y=" << y << endl;
    }
};

//线
class Line
{

public:
    double a, b, c;
    Line()
    {
        a = 0;
        b = 0;
        c = 0;
    }
    Line(double _a, double _b, double _c)
    {
        a = _a;
        b = _b;
        c = _c;
    }
    Line(Point p1, Point p2)
    {
        if (p1.y == p2.y)
        {
            a = 0;
            b = 1;
            c = -p1.y;
        }
        else if (p1.x == p2.x)
        {
            a = 1;
            b = 0;
            c = -p1.x;
        }
        else
        {
            a = 1;
            b = (p2.x - p1.x) / (p1.y - p2.y);
            c = -p1.x - b * p1.y;
        }
    }
    Line(Point p1, Vector vec)
    {
        if (vec.x == 0)
        {
            b = 0;
            a = 1;
            c = -p1.x;
        }
        else if (vec.y == 0)
        {
            a = 0;
            b = 1;
            c = -p1.y;
        }
        else
        {
            Point temp(p1.x + vec.x, p1.y + vec.y);
            a = 1;
            b = (temp.x - p1.x) / (p1.y - temp.y);
            c = -p1.x - b * p1.y;
        }
    }
    void showSelf()
    {
        cout << "a=" << a << " b=" << b << " c=" << c << endl;
    }
};

bool goStraight(PlaneStruct *plane, Point p, KeyStruct *key);

//求两点之间的距离
double distance(Point p1, Point p2)
{
    return (p1 - p2).length();
}

//求绝对值
double fabs(double x)
{
    if(x>=0)
        return x;
    else
        return -x;
}

double angleConver(double angle)
{
    return -90 - angle;
}

//通过递归计算余数，是否需要优化待检测
double get_fmod(double x, double y)
{
    if (x < y)
    {
        return x;
    }
    else if (x - y <= 0.001)
    {
        return 0;
    }
    else
    {
        return get_fmod(x - y, y);
    }
}

//平方
double TwoTimes(double x)
{
    return x*x;
}

//返回当前飞机对应的角度，不大于360°
double get_plane_angle_now(PlaneStruct* plane)
{
    //飞机20ms能转动3.6°，完成一圈则是2s
    double angle=plane->angle;
    if(angle<0)
    {
        while(angle<0)
        {
            angle+=360;
        }
    }
    return get_fmod(angle,360);
}

//求两向量之间的余弦值
double cosTwoVec(Vector v1, Vector v2)
{
    return (v1 * v2) / ((v1.length()) * v2.length());
}

//求解一元二次方程
bool rootOfQuadraticEquation(double a, double b, double c, double& big, double& small)
{
    double dai = b * b - 4 * a * c;
    if (dai >= 0)
    {
        if (a > 0)
        {
            big = (-b + sqrt(dai)) / (2 * a);
            small = (-b - sqrt(dai)) / (2 * a);
            return true;
        }
        else if (a < 0)
        {
            small = (-b + sqrt(dai)) / (2 * a);
            big = (-b - sqrt(dai)) / (2 * a);
            return true;
        }
        else if(a==0)
        {
            big=small=-b/c;
        }
    }
    else
        return false;
}

//返回提前瞄准对应的飞机子弹飞行时间
double solveAimPreQuestion(PlaneStruct*plane,BallStruct*ball)
{
    double t1,t2;
    double der_x,der_y,v,v0=2000,x1,y1,x0,y0;
    double v_x,v_y;
    x0=plane->x;
    y0=plane->y;
    x1=ball->x;
    y1=ball->y;
    v_x=ball->v_x;
    v_y=ball->v_y;
    der_x=x1-x0;
    der_y=y1-y0;
    v=sqrt(TwoTimes(v_x)+TwoTimes(v_y));
    if(rootOfQuadraticEquation(TwoTimes(v)-TwoTimes(v0),2*(der_x*v_x+der_y*v_y),TwoTimes(der_x)+TwoTimes(der_y),t1,t2))
    {
        if(t1==t2&&t1>=0)
            return t1;
        else if(t1>t2&&t2>=0)
            return t2;
        else if(t1>=0&&t2<=0)
            return t1;
        else
            return -1;
    }
    else
        return -1;  //无法提前瞄准
}

//点到线的距离
double disPointToLine(Point& p, Line& line)
{
    return fabs(line.a * p.x + line.b * p.y + line.c) / sqrt(line.a * line.a + line.b * line.b);
}

//返回点到直线的垂点
Point findCrossedPoint(Point& p, Line& line)
{
    Point temp;
    temp.x = (line.b * line.b * p.x - line.a * line.b * p.y - line.a * line.c) / (line.a * line.a + line.b + line.b);
    temp.y = line.b / line.a * (temp.x - p.x) + p.y;
    return temp;
}

//求解p1关于p2的对称点
Point findSymmetryPoint(Point& p1, Point& p2)
{
    Point p;
    p.x = 2 * p2.x - p1.x;
    p.y = 2 * p2.y - p1.y;
    return p;
}

//瞄准某个点
bool aimPoint(PlaneStruct* plane, Point* p, KeyStruct* key)
{
    //飞机到目的地的角度
    double angleTo, angleDiff;

    //计算飞机到目的地的角度并改变坐标系
    angleTo = angleConver(atan2(p->y - plane->y, p->x - plane->x) * 180 / PI);
    //计算飞机朝向与该角度之差
    angleDiff = fmod(fmod(plane->angle - angleTo, 360) + 360, 360);

    //根据角度差选择更优旋转方向
    if (angleDiff < 1.5 || angleDiff > 359)
    {
        return 1;
    }
    else if (angleDiff < 180)
    {
        key->rotate_left = false;
        key->rotate_right = true;
    }
    else
    {
        key->rotate_left = true;
        key->rotate_right = false;
    }

    return 0;
}

//瞄准某个球，待修改
void aimBall(PlaneStruct*plane,BallStruct*ball,KeyStruct*key)
{
    Point ballPos(ball->x,ball->y);
    aimPoint(plane,&ballPos,key);
}

//到达某个点
bool arrivePoint(PlaneStruct*plane,Point *pos)
{
    Point planePos(plane->x,plane->y);
    if(distance(planePos,*pos)<50)
        return true;
    else
        return false;
}

//判断是否处于某个球带来的危险中
bool isDanger(PlaneStruct*plane,BallStruct*ball)
{
    //基础数据
    Point ballPos(ball->x,ball->y);
    Vector ballVec(ball->v_x,ball->v_y);
    Line ballLine(ballPos,ballVec);
    Point planePos(plane->x,plane->y);
    double dis=disPointToLine(planePos,ballLine);   //飞机到球运行轨迹的距离
    Vector planeToBall=planePos-ballPos;
    double cos=cosTwoVec(ballVec,planeToBall);
    if(cos<=0&&dis<=plane->r+ball->r+20)
        return true;
    else
        return false;
}

//是否完全安全
bool isAllSafe(DataStruct*data)
{
    int i;
    for(i=0;i<data->ball_size;i++)
    {
        if(isDanger(&data->plane1,&data->ball[i]))
            return false;
    }
    return true;
}

//逃离某个球
bool escapeBall(PlaneStruct* plane,BallStruct* ball,KeyStruct* key)
{
    double ball_v=sqrt(ball->v_x*ball->v_x+ball->v_y*ball->v_y);
    Point plane_pos(plane->x,plane->y);
    Point ball_pos(ball->x,ball->y);
    Vector ballVec(ball->v_x,ball->v_y);
    Line ballLine(ball_pos,ballVec);
    if(ball_v>2000&&distance(plane_pos,ball_pos)-plane->r-ball->r<200)
        key->forward=1; //球速太快，拔腿就跑
    else if(distance(plane_pos,ball_pos)-plane->r-ball->r<10)
    {
        key->forward=1;
    }
    else
    {
        Point crossedPoint=findCrossedPoint(plane_pos,ballLine);
        Point sysPoint=findSymmetryPoint(crossedPoint,plane_pos);
        bool isFinishedAimPoint = aimPoint(plane,&sysPoint,key);
        if(isFinishedAimPoint)
        {
            key->forward=1;
        }
        else
        {
            key->forward=0;
        }
    }
    if(!isDanger(plane,ball))
    {
        return true;    //成功逃离
    }
    else
    {
        return false;
    }
}

//笔直的冲向某点，不需要多余操作
bool goStraight(PlaneStruct*plane,Point p,KeyStruct*key)
{
    if(aimPoint(plane,&p,key))
    {
        if(!arrivePoint(plane,&p))
        {
            startSituation(key);
            key->forward=1;
        }
    }
    return arrivePoint(plane,&p);
}

//判断是否被边界卡住
bool isStucked(DataStruct*data,KeyStruct*key)
{
    double planeAngle=get_plane_angle_now(&data->plane1);
    double plane_x=data->plane1.x;
    double plane_y=data->plane1.y;
    int isGo=key->forward;
    if(isGo)
    {
        if(plane_y==1450&&planeAngle>=100&&planeAngle<=260)
            return true;
        else if(plane_x==50&&planeAngle>=10&&planeAngle<=170)
            return true;
        else if(plane_x==1950&&planeAngle>=190&&planeAngle<=350)
            return true;
        else if(plane_y==50&&((planeAngle>=0&&planeAngle<=80)||(planeAngle<=359.9&&planeAngle>=280)))
            return true;
        else
            return false;
    }
    return false;
}

//找寻对自己有危险的球，距离最近的那一个
BallStruct& findDangerBall(DataStruct*data)
{
    int i,count=0,minIndex=0;
    int danV[30];
    double minDis=2000,dis=0; //记录最小距离
    Point planePos(data->plane1.x,data->plane1.y);
    for(i=0;i<data->ball_size;i++)
    {
        if(isDanger(&data->plane1,&data->ball[i]))
        {
            danV[count++]=i;
        }
    }
    for(i=0;i<count;i++)
    {
        Point ballPos(data->ball[danV[i]].x,data->ball[danV[i]].y);
        dis=distance(planePos,ballPos);
        if(dis<minDis)
        {
            minDis=dis;
            minIndex=danV[i];
        }
    }
    return data->ball[minIndex];
}

//找寻最大球
BallStruct& findMaxBall(DataStruct*data)
{
    int i,maxIndex=0,maxR=0;

    for(i=0;i<data->ball_size;i++)
    {
        if(data->ball[i].r>maxR)
        {
            maxR=data->ball[i].r;
            maxIndex=i;
        }
    }
    return data->ball[maxIndex];
}

//找寻最近球
BallStruct& findClosedBall(DataStruct*data)
{
    int i,minIndex=0;
    double minDis=2000;
    Point planePos(data->plane1.x,data->plane1.y);
    for(i=0;i<data->ball_size;i++)
    {
        Point ballPos(data->ball[i].x,data->ball[i].y);
        double dis=distance(planePos,ballPos);
        if(dis<minDis)
        {
            minIndex=i;
            minDis=dis;
        }
    }
    return data->ball[minIndex];
}

//前往某个点
bool goToPoint(DataStruct*data,PlaneStruct*plane,Point *p,KeyStruct*key)
{
    aimPoint(plane,p,key);
    if(arrivePoint(plane,p))
        return true;
    else
    {
        if(isAllSafe(data))
        {
            key->forward=1;
        }
        else
        {
            escapeBall(&data->plane1,&findDangerBall(data),key);
        }
        return false;
    }
}

//返回球的预测位置
Point getPreBallPos(PlaneStruct*plane,BallStruct*ball)
{
    double t=solveAimPreQuestion(plane,ball);
    if(t!=-1)
    {
        Point preBall(ball->x+ball->v_x*t,ball->y+ball->v_y*t);
        return preBall;
    }
    else
    {
        Point p(-1,-1);
        return p;
    }
}

//判断点是否在屏幕内
bool isInGameScreen(Point& p)
{
    if(p.x>=0&&p.x<=2000&&p.y>=0&&p.y<=1500)
        return true;
    else
        return false;
}

//提前瞄准球，核心代码
bool aimPreBall(PlaneStruct*plane,BallStruct*ball,KeyStruct*key)
{
    double t=solveAimPreQuestion(plane,ball);
    if(t!=-1)
    {
        Point preBall(ball->x+ball->v_x*t,ball->y+ball->v_y*t);       
        if(isInGameScreen(preBall))
        {
            if(aimPoint(plane,&preBall,key))
            {
                return true;
            }
        }
    }
    return false;
}

//找到第一个在屏幕内的球
BallStruct& findInGameScreenBall(DataStruct*data)
{
    int i;
    for(i=0;i<data->ball_size;i++)
    {
        Point p=getPreBallPos(&data->plane1,&data->ball[i]);
        if(isInGameScreen(p))
        {
            return data->ball[i];
        }
    }
    return data->ball[0];
}

//非常危险
bool isVeryDanger(PlaneStruct*plane,BallStruct*ball)
{
    Point planePos(plane->x,plane->y);
    Point ballPos(ball->x,ball->y);
    if(distance(planePos,ballPos)<=plane->r+ball->r+10)
        return true;
    else
        return false;
}

//初始化
void startSituation(KeyStruct* key)
{
    key->forward = 0;
    key->rotate_left = 0;
    key->rotate_right = 0;
    key->shoot =0;
}


//每隔10ms调用一次
void ai_pve(DataStruct* data, KeyStruct* key)
{
    startSituation(key);
    BallStruct& closeBall=findClosedBall(data);
    if(isAllSafe(data))
    {
        bool isok=aimPreBall(&data->plane1,&closeBall,key);
        if(isok)
            key->shoot=1;
        else
        {
            aimPreBall(&data->plane1,&data->ball[0],key);
            key->shoot=1;
        }
    }
    else
    {
        BallStruct dangerBall=findDangerBall(data);
        if(isVeryDanger(&data->plane1,&dangerBall))
            key->shield=1;
        aimPreBall(&data->plane1,&dangerBall,key);
        //escapeBall(&data->plane1,&findDangerBall(data),key);
        key->shoot=1;
    }
}














