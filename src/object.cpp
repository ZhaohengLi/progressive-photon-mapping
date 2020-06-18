//
// Created by 李曌珩 on 2019-06-23.
//
#include "object.h"
#include <cmath>
#include <cassert>
#include <iostream>
#include <random>

BazierCurve::BazierCurve(int id) : id(id) {
}

void BazierCurve::accept(const Json::Value& val) {
    Object::accept(val);
    Vector _Q;
    _Q.accept(val["position"]);
    Q = _Q.eigen();
    for (int i=0;i<4;i++) {
        px[i] = val["ctrl_pts"][id][0][i].asDouble();
        py[i] = val["ctrl_pts"][id][1][i].asDouble();
    }
    std::cout<<id<<" FINISH"<<std::endl;
    maxX = -1e100;
    minX = 1e100;
    maxR = -1e100;
    minR = 1e100;
    for (int i=0;i<4;i++) {
        maxX = std::max(maxX,py[i]);
        minX = std::min(minX,py[i]);
        maxR = std::max(maxR,px[i]);
        minR = std::min(minR,px[i]);
    }
}

Color BazierCurve::getColor(const Vector&)const {
    if (texture->getType() != TEXTURE_PURE_COLOR)
        std::cout<<"The getColor of BazierCurve only support PURE_COLOR_MODE"<<std::endl;
    return texture->getColor();
}
int BazierCurve::cylinderCollision(const Vector& _rayO,const Vector& rayD,double r) {
    Vector rayO = _rayO - Q;
    double ox,oy,dx,dy;
    ox = rayO.getY();
    oy = rayO.getZ();
    dx = rayD.getY();
    dy = rayD.getZ();
    double a,b,c;
    a = dx*dx+dy*dy;
    b = 2*(dx*ox+oy*dy);
    c = ox*ox+oy*oy - r*r;
    double delta = b*b-4*a*c;
    if (delta<=0)
        return 0;
    double k1 = (-b-sqrt(delta))/(2*a);
    double k2 = (-b+sqrt(delta))/(2*a);
    double x1 = rayO.getX() + k1 * rayD.getX();
    double x2 = rayO.getX() + k2 * rayD.getX();
    if (x1>x2)std::swap(x1,x2);
    if (x2<minX || x1>maxX) {
        return 0;
    }
    if (x1<minX && x2>maxX)
        return 2;//竖直射入
    if (x1>minX && x2<maxX)
        return 1;//水平射入
    return 3;
}
bool BazierCurve::checkCollision(const Vector& rayO,const Vector& rayD) {
    int max_s = cylinderCollision(rayO,rayD,maxR);
    int min_s = cylinderCollision(rayO,rayD,minR);
    if (max_s == 0 || min_s == 2)
        return false;
    return true;
}
bool BazierCurve::initArgs(double &t,double &u,double &theta,Vector rayO,Vector rayD,double _u = -1) {
    if (_u>=0)
        u = _u;
    else
        u = rand()*1.0/RAND_MAX;
    double h = rand()*1.0/RAND_MAX *(maxX-minX)+maxX;
    t = ((Q.x() + h) - rayO.getX()) / rayD.getX();
    Vector C = rayO + t*rayD;
    Vector V = (Q+Vector(h,0,0) - C);
    assert(abs(V.getX())<feps);
    theta = rand()*1.0/RAND_MAX*M_PI*2;
    if (V.getY()*V.getY() + V.getZ()*V.getZ() > maxR*maxR)
        return true;
    return true;
}

bool BazierCurve::collideWith(const Vector& rayO,const Vector& rayD,Collision& collision) {
    Eigen::Vector3d O = rayO.eigen();
    Eigen::Vector3d D = rayD.eigen();
    collision.dist = 1e100;
    if (!checkCollision(rayO,rayD))
        return false;
    for (int cnt=0;cnt<35 ;cnt++){
        double lr = .7;
        double t;
        double u;
        double theta;
        if (!initArgs(t,u,theta,rayO,rayD,cnt<2?cnt:-1)) {
            continue;
        }
        Eigen::Vector3d args(t,u,theta);
        bool flag = false;
        for (int iter=0;iter<20;iter++) {
            t = args.x();
            u = args.y();
            theta = args.z();
            if (u<-.5 || u>1.5)break;
            Eigen::Vector3d F = getF(t,u,theta,O,D);
            Eigen::Matrix3d dF = getdF(t,u,theta,O,D);
            if (std::max(std::max(std::abs(F.x()),std::abs(F.y())),std::abs(F.z())) < 1e-7) {
                flag = true;
                break;
            }
            args = args - (dF.inverse()*F)*lr;
        }
        if (!flag)continue;
        if (t<0)continue;
        if (u<0 || u>1)continue;
        if (t > collision.dist)continue;
        collision.C = rayO + t*rayD;
        collision.dist = t;
        Eigen::Vector3d pspu(getdP(py,u), sin(theta)*getdP(px,u),cos(theta)*getdP(px,u));
        Eigen::Vector3d pspt(0, cos(theta)*getP(px,u) , -sin(theta)*getP(px,u));
        Eigen::Vector3d _N = pspu.cross(pspt);
        collision.N = Vector(_N).unit();
        collision.face = (collision.N ^ rayD.reverse()) > 0;
        if (!collision.face)
            collision.N = collision.N.reverse();
        collision.I = rayD;
        collision.belongs = this;
    }
    if (collision.dist < 1e90){return true;}
    return false;
}

inline double BazierCurve::getP(double* p,double t) {
    return 1*p[0]*(1-t)*(1-t)*(1-t) +
           3*p[1]*t*(1-t)*(1-t) +
           3*p[2]*t*t*(1-t) +
           1*p[3]*t*t*t;
}

inline double BazierCurve::getdP(double* p,double t) {
    return -3*p[0]*(1-t)*(1-t) +
           3*p[1]*(1-t)*(1-t) +
           -6*p[1]*t*(1-t) +
           6*p[2]*(1-t)*t +
           -3*p[2]*t*t +
           3*p[3]*t*t;
}

Eigen::Vector3d BazierCurve::getC(double t,const Eigen::Vector3d& O,const Eigen::Vector3d& D) {
    return O + D*t;
}

Eigen::Vector3d BazierCurve::getS(double u,double theta) {
    return Q + Eigen::Vector3d( getP(py,u), sin(theta)*getP(px,u), cos(theta)*getP(px,u));
}

Eigen::Vector3d BazierCurve::getF(double t,double u,double theta, const Eigen::Vector3d& O,const Eigen::Vector3d& D) {
    return getC(t,O,D) - getS(u,theta);
}

Eigen::Matrix3d BazierCurve::getdF(double t,double u,double theta, const Eigen::Vector3d& O,const Eigen::Vector3d& D) {
    Eigen::Matrix3d res;
    res <<
        D.x() , -getdP(py,u) , 0,
            D.y() , -sin(theta)*getdP(px,u) , -cos(theta)*getP(px,u) ,
            D.z() , -cos(theta)*getdP(px,u) , +sin(theta)*getP(px,u) ;
    return res;
}



Sphere::Sphere() {
}

void Sphere::accept(const Json::Value& val) {
    Object::accept(val);
    O.accept(val["center"]);
    radius = val["radius"].asDouble();
}

Color Sphere::getColor(const Vector&v)const {
    if (texture->getType() == TEXTURE_PURE_COLOR)
    {
        return texture->getColor();
    }else if (texture->getType() == TEXTURE_PICTURE) {
        Vector tmp = (v - O)/radius;
        if (!tmp.isUnit())
            std::cout<<"The vector is not on the surface!"<<std::endl;
        double x = asin(tmp.getZ())+M_PI/2;
        double y = atan2(tmp.getX(),tmp.getY());
        Color res = texture->getColor(x,y);
        return res;
    }
    assert(false);
}

bool Sphere::collideWith(const Vector& rayO,const Vector& rayD,Collision& collision) {
    assert(rayD.isUnit());
    Vector V = O-rayO;
    double c = V.sqrlen()-radius*radius;
    double b = -(V^rayD)*2;
    double a = rayD.sqrlen();
    double delta = b*b-4*a*c;
    if (delta > feps) {
        double d1 = (-b-sqrt(delta)) / (2*a);
        double d2 = (-b+sqrt(delta)) / (2*a);
        collision.dist = d1;
        if (d1 < 0 && d2 < 0){
            return false;
        }else if (d1 < 0 && d2 >= 0) {
            collision.dist = d2;
            collision.face = false;
        }else {
            collision.dist = d1;
            collision.face = true;
        }
        collision.belongs = this;
        collision.C = rayO + rayD * collision.dist;
        if (collision.face)
            collision.N = (collision.C - O).unit();
        else
            collision.N = (O-collision.C).unit();
        Vector NN = collision.N * (collision.N ^ rayD.reverse());
        collision.I = rayD;
        return true;
    }else {
        return false;
    }
}

Plane::Plane() {
}
Plane::Plane(Vector O,Vector dx,Vector dy):O(O),dx(dx),dy(dy) {
}

void Plane::accept(const Json::Value& val) {
    Object::accept(val);
    O.accept(val["position"]);
    dx.accept(val["dx"]);
    dy.accept(val["dy"]);
    border = val["border"].asBool();
}

Color Plane::getColor(const Vector& pos)const {
    if (texture->getType() == TEXTURE_PURE_COLOR) {
        return texture->getColor();
    }else if (texture->getType() == TEXTURE_PICTURE) {
        double tx = ((pos - O)^dx)/dx.len();
        double ty = ((pos - O)^dy)/dy.len();
        return texture->getColor(tx,ty);
    }else
    {
        std::cout<<"The getColor of Plane only support PURE_COLOR_MODE"<<std::endl;
        return Color();
    }
}

bool Plane::collideWith(const Vector& rayO,const Vector& rayD,Collision& collision) {
    assert(rayD.isUnit());
    Vector N = (dx*dy).unit();
    double d = -(N^O);
    double t = -(d+(N^rayO))/(N^rayD);
    if (!std::isfinite(t))return false;
    if (t < 0) {
        return false;
    }
    collision.belongs = this;
    collision.dist = t;
    collision.C = rayO + t*rayD;
    collision.face = (rayD^N) < 0;//True : front face, the ray hit toward to plane
    double r1 = ((collision.C - O)^dx)/dx.sqrlen();
    double r2 = ((collision.C - O)^dy)/dy.sqrlen();
    if (border && (r1>1-feps || r2>1-feps || r1<feps || r2<feps) ){
        return false;
    }
    collision.N = collision.face ? N : -N;
    collision.I = rayD;
    return true;
}


void PointLight :: accept(const Json::Value& val) {
    Light::accept(val);
    O.accept(val["position"]);
}

bool PointLight :: collideWith(const Vector& rayO, const Vector& rayD,Collision& collision) {
    return false;
}

Vector PointLight::getCenter()const {
    return O;
}

double PointLight::getShade(const Vector& _rayO,std::vector<Object*> olist, int shade_quality)const {
    Vector rayO = _rayO;
    int success_count = 0;
    Vector checkO = rayO;
    Vector checkT = O;
    Vector checkD = (checkT - rayO).unit();
    double dist = (checkT - rayO).len();
    bool flag = true;
    for (auto &w: olist)
    {
        bool flg;
        Collision obj_coll;
        if ((flg = w->collideWith(checkO,checkD,obj_coll)) && obj_coll.dist < dist) {
            return 0;
        }
    }
    return 1;
}
void PointLight::randomlyEmit(Vector& rayO,Vector& rayD)const{
    rayO = O;
    rayD = Vector::randomVectorOnSphere();
}
