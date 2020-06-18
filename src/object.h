//
// Created by 李曌珩 on 2019-06-23.
//

#ifndef CODE_OBJECT_H
#define CODE_OBJECT_H

#include "core.h"
#include "Eigen/Dense"

class BazierCurve : public Object {
private:
    Eigen::Vector3d Q;
    Color color;
    int id;
    double px[4];
    double py[4];
    double maxX,minX;
    double maxR,minR;
public:
    BazierCurve(int id);
    virtual void accept(const Json::Value& val);
    Color getColor(const Vector&)const;
    bool collideWith(const Vector& rayO,const Vector& rayD,Collision& collision);
    double getP(double* p,double t);
    double getdP(double* p,double t);
    Eigen::Vector3d getC(double t,const Eigen::Vector3d& O,const Eigen::Vector3d& D);
    Eigen::Vector3d getdC(double t);
    Eigen::Vector3d getS(double u,double theta);
    Eigen::Vector3d getF(double t,double u,double theta,const Eigen::Vector3d& O,const Eigen::Vector3d& D);
    Eigen::Matrix3d getdF(double t,double u,double theta,const Eigen::Vector3d& O,const Eigen::Vector3d& D);
    bool initArgs(double &t,double &u,double &theta,Vector rayO,Vector rayD,double _u);
    bool checkCollision(const Vector& rayO,const Vector& rayD);
    int cylinderCollision(const Vector& rayO,const Vector& rayD,double r);
};

class Sphere : public Object {
private:
    Vector O;
    double radius;
public:
    Sphere();
    virtual void accept(const Json::Value& val);
    Color getColor(const Vector&)const;
    bool collideWith(const Vector& rayO,const Vector& rayD,Collision& collision);
};

class Plane : public Object {
private:
    Vector O;
    Vector dx,dy;
    bool border;
public:
    Plane();
    Plane(Vector O,Vector dx,Vector dy);
    void accept(const Json::Value& val);
    Color getColor(const Vector&)const;
    bool collideWith(const Vector& rayO,const Vector& rayD,Collision& collision);
};


class PointLight : public Light {
private:
    Vector O;
public:
    PointLight(){}
    void accept(const Json::Value& val);
    bool collideWith(const Vector& rayO, const Vector& rayD, Collision& collision);
    Vector getCenter()const;
    double getShade(const Vector& rayO,std::vector<Object*> olist,int shade_quality)const;
    void randomlyEmit(Vector& rayO,Vector& rayD)const;
};


#endif //CODE_OBJECT_H
