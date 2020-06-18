//
// Created by 李曌珩 on 2019-06-23.
//

#ifndef CODE_CORE_H
#define CODE_CORE_H

#include "json/json.h"
#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define TEXTURE_PURE_COLOR 1
#define TEXTURE_PICTURE 2
#define INF 0x3f3f3f3f
#define inf 1e100
#define feps 1e-7

class Vector
{
    double x,y,z;
public:
    Vector();
    Vector(double x,double y,double z);
    Vector(const Eigen::Vector3d vec);
    double getX() const {return x;}
    double getY() const {return y;}
    double getZ() const {return z;}
    virtual void accept(const Json::Value &sl);
    friend Vector operator * (const Vector &a,const Vector &b);
    friend double operator ^ (const Vector &a,const Vector &b);
    friend Vector operator + (const Vector &a,const Vector &b);
    friend Vector operator - (const Vector &a,const Vector &b);
    friend Vector operator - (const Vector &a);
    friend Vector operator * (double k,const Vector &a);
    friend Vector operator * (const Vector &a,double k);
    friend bool operator == (const Vector &a,const Vector &b);
    friend Vector operator /(const Vector& a,double k);
    friend Vector each_min(const Vector& v1,const Vector& v2);
    friend Vector each_max(const Vector& v1,const Vector& v2);
    Vector unit()const;
    Vector reverse()const;
    double len()const;
    double sqrlen()const;
    bool isUnit()const;
    std::string description()const;
    Eigen::Vector3d eigen()const;
    static Vector randomVectorOnSphere();
};



class Texture
{
public:
    Texture();
    virtual ~Texture();
    virtual Color getColor(double dx=0,double dy=0)const = 0;
    virtual void accept(const Json::Value& val) = 0;
    virtual int getType() = 0;
};

class PureColorTexture : public Texture {
private:
    Color color;
public:
    PureColorTexture();
    ~PureColorTexture();
    Color getColor(double,double)const;
    int getType(){return TEXTURE_PURE_COLOR;}
    void accept(const Json::Value& val);
};

class PictureTexture : public Texture {
private:
    std::string filename;
    double rx,ry;
    cv::Mat image;
public:
    PictureTexture();
    ~PictureTexture();
    Color getColor(double,double)const;
    int getType(){return TEXTURE_PICTURE;}
    void accept(const Json::Value& val);
};

class Object;

struct Collision {
    Vector C;//The Center of Collision
    Vector N;//The normal of the plane
    Vector I;//The direction of reflaction
    double dist;
    bool face;
    Object* belongs;
    std::string description()const;
    void refraction(Vector& resO, Vector& resD)const;
    void reflection(Vector& resO, Vector& resD)const;
    void diffusion(Vector& resO, Vector& resD)const;
    void diffusion_hl(Vector& resO, Vector& resD)const;
    Vector getSurfaceC()const;
    Vector getBackfaceC()const;
};

struct Material {
    double refl;//reflection ratio
    double diff;//diffusion ratio
    double spec;//high light diffusion
    double refr;//refraction ratio
    double refr_k;
    void accept(const Json::Value& val);
};

class Object {
protected:
    unsigned hash;
    Material material;
    std::string name;
    Texture* texture;
    Texture* absorb;
public:
    Object();
    virtual ~Object();
    virtual void accept(const Json::Value& val);
    unsigned getHash()const {return hash;}

    const Material& getMaterial()const {return material;}

    virtual bool collideWith(const Vector& rayO, const Vector& rayD,Collision& coll) = 0;
    virtual Color getColor(const Vector &pos)const = 0;
    std::string getName()const{return name;}
    const Texture& getAbsorb()const;
};


class Light : public Object {
protected:
    double brightness;
public:
    Light();
    virtual ~Light();
    virtual void accept(const Json::Value& val);
    virtual Vector getCenter()const = 0;
    virtual double getShade(const Vector& rayO,std::vector<Object*> olist,int shade_quality)const = 0;
    virtual void randomlyEmit(Vector& rayO,Vector& rayD)const = 0;
    double getBrightness()const {return brightness;}
    Color getColor(const Vector&)const;
};


class Color {
private:
    double r,g,b;
public:
    Color();
    Color(double r,double g,double b);
    Color operator += (const Color&a);
    void accept(const Json::Value& val);
    double getR()const {return r;}
    double getG()const {return g;}
    double getB()const {return b;}
    friend Color operator +(const Color& a,const Color &b);
    friend Color operator *(const Color& a,double k);
    friend Color operator *(const Color& a,const Color& b);
    Color exp()const;
    std::string description()const;
    Color adjust()const;
};

class Camera {
private:
    Vector position;
    Vector dx,dy;
    Vector origin;
    int rx,ry;
    double fdepth;
public:
    Camera(int rx,int ry);
    ~Camera();
    void accept(const Json::Value& val);
    void getRay(double scanX, double scanY, Vector &rayO, Vector &rayD);
    void addDepth(const Vector& rayO,const Vector& rayD,Vector& resO,Vector& resD);
};


#endif //CODE_CORE_H
