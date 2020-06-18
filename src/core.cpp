//
// Created by 李曌珩 on 2019-06-23.
//

#include "core.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <random>
#include <cstdlib>
#include <algorithm>

using namespace std;
using namespace cv;


Vector :: Vector() {
    x = 0 , y = 0, z = 0;
}

Vector :: Vector(double x,double y,double z) : x(x), y(y), z(z){
}

Vector :: Vector(const Eigen::Vector3d vec) : x(vec(0,0)),y(vec(1,0)),z(vec(2,0)) {
}

void Vector :: accept(const Json::Value& val) {
    if (!val.isMember("x") || !val.isMember("y") || !val.isMember("z"))
        std::cout<<"The vector not found..."<<std::endl;
    x = val["x"].asDouble();
    y = val["y"].asDouble();
    z = val["z"].asDouble();
}

Vector operator*(const Vector &a,const Vector &b){
    return Vector(a.y*b.z - a.z*b.y,a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

double operator^(const Vector &a,const Vector &b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vector operator +(const Vector &a,const Vector &b) {
    return Vector(a.x+b.x,a.y+b.y,a.z+b.z);
}

Vector operator -(const Vector &a,const Vector &b) {
    return Vector(a.x-b.x,a.y-b.y,a.z-b.z);
}
Vector operator -(const Vector &a) {
    return Vector(-a.x,-a.y,-a.z);
}

Vector operator *(double k,const Vector &a) {
    return Vector(k*a.x,k*a.y,k*a.z);
}

Vector operator *(const Vector &a,double k) {
    return Vector(k*a.x,k*a.y,k*a.z);
}

Vector operator /(const Vector&a, double k) {
    return Vector(a.x/k,a.y/k,a.z/k);
}


bool operator ==(const Vector &a,const Vector &b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

double Vector::len()const {
    return sqrt(x*x+y*y+z*z);
}
double Vector::sqrlen()const {
    return x*x+y*y+z*z;
}
Vector Vector::unit()const {
    return (*this)/len();
}
Vector Vector::reverse()const {
    return Vector(-x,-y,-z);
}
bool Vector::isUnit()const {
    return abs(x*x+y*y+z*z - 1) < feps;
}

Vector each_max(const Vector& v1,const Vector& v2) {
    return Vector(std::max(v1.x,v2.x),std::max(v1.y,v2.y),std::max(v1.z,v2.z));
}

Vector each_min(const Vector& v1,const Vector& v2) {
    return Vector(std::min(v1.x,v2.x),std::min(v1.y,v2.y),std::min(v1.z,v2.z));
}

std::string Vector::description()const {
    std::stringstream ss;
    std::string res;
    ss<<"Vector<"<<x<<","<<y<<","<<z<<">";
    ss>>res;
    return res;
}
Eigen::Vector3d Vector::eigen()const {
    return Eigen::Vector3d(x,y,z);
}

Vector Vector::randomVectorOnSphere() {
    double theta = rand()*1.0/RAND_MAX*M_PI*2;
    double phi = rand()*1.0/RAND_MAX*M_PI*2 - M_PI;
    return Vector(cos(phi)*sin(theta),cos(phi)*cos(theta),sin(phi));
}


Texture::Texture() {
}
Texture::~Texture() {
}

PureColorTexture::PureColorTexture() {
}
PureColorTexture::~PureColorTexture() {
}

Color PureColorTexture::getColor(double,double)const {
    return color;
}

void PureColorTexture::accept(const Json::Value& val) {
    color.accept(val);
}

PictureTexture::PictureTexture() {
}
PictureTexture::~PictureTexture() {
}

Color PictureTexture::getColor(double x,double y)const {
    int ix = (int(floor(fmod(x,rx)/rx * image.rows)) + image.rows)%image.rows;
    int iy = (int(floor(fmod(y,ry)/ry * image.cols)) + image.cols)%image.cols;
    assert(ix<image.rows && iy < image.cols);
    double r = image.at<Vec3b>(ix, iy)[0]/255.0;
    double g = image.at<Vec3b>(ix, iy)[1]/255.0;
    double b = image.at<Vec3b>(ix, iy)[2]/255.0;
    return Color(r,g,b);
}

void PictureTexture::accept(const Json::Value& val) {
    filename = val["filename"].asString();
    rx = val["rx"].asDouble();
    ry = val["ry"].asDouble();
    image = cv::imread(filename.c_str());
    std::cout<<"Imaged <"<<filename<<"> read "<<image.rows<<" "<<image.cols<<std::endl;
}

Object::Object() {
    hash = rand();
}

Object::~Object() {
    if (texture)
        delete texture;
    if (absorb)
        delete absorb;
}

void Collision::refraction(Vector& resO, Vector& resD)const {
    double cosI = N.reverse()^I;
    double n = face ? 1.0/belongs->getMaterial().refr_k : belongs->getMaterial().refr_k;
    double cosT2 = 1 - (n*n)*(1-cosI*cosI);
    if (cosT2 > feps) {
        resO = getBackfaceC();
        resD = I*n + N*(n*cosI - sqrt(cosT2));
    }else {
        reflection(resO,resD);
    }
}

void Collision::reflection(Vector& resO, Vector& resD)const {
    Vector _N = N*(N^I.reverse());
    resD = (_N*2-I.reverse());
    resO = getSurfaceC();
}

void Collision::diffusion(Vector& resO, Vector& resD)const {
    Vector DD = (N*Vector::randomVectorOnSphere()).unit();
    assert(DD.isUnit());
    double phi = acos(rand()*1.0/RAND_MAX);
    resO = getSurfaceC();
    resD = DD*sin(phi) + N*cos(phi);
    assert(resD.isUnit());
}

void Collision::diffusion_hl(Vector& resO, Vector& resD)const {
}

Vector Collision::getSurfaceC()const {
    return C+N*(feps*2);
}
Vector Collision::getBackfaceC()const {
    return C-N*(feps*2);
}

void Material::accept(const Json::Value& val) {
    if (!val.isMember("refl"))
        std::cout<<"No refl found in material..."<<std::endl;
    refl = val["refl"].asDouble();
    if (!val.isMember("spec"))
        std::cout<<"No spec found in material..."<<std::endl;
    spec = val["spec"].asDouble();
    if (!val.isMember("diff"))
        std::cout<<"No diff found in material..."<<std::endl;
    diff = val["diff"].asDouble();
    if (!val.isMember("refr"))
        std::cout<<"No refr found in material..."<<std::endl;
    refr = val["refr"].asDouble();
    if (!val.isMember("refr_k"))
        std::cout<<"No refr_k found in material..."<<std::endl;
    refr_k = val["refr_k"].asDouble();
}

void Object::accept(const Json::Value& val) {
    name = val["name"].asString();
    if (!val.isMember("texture")) {
        std::cout<<"No texture info find"<<std::endl;
    }
    if (val["texture"]["type"] == "pure")
        texture = new PureColorTexture();
    else if (val["texture"]["type"] == "picture")
        texture = new PictureTexture();
    else
        std::cout<<"Invalid Texture Type..."<<std::endl;
    texture->accept(val["texture"]);
    material.accept(val["material"]);
    if (material.refr>feps)
    {
        absorb = new PureColorTexture();
        absorb->accept(val["absorb"]);
    }
}

std::string Collision::description()const {
    static char buf[20];
    sprintf(buf,"%f",dist);
    return std::string("Collision : \n") +
           "C = "+C.description() +"\n" +
           "N = " + N.description() + "\n" +
           "I = " + I.description() + "\n" +
           "dist = " + std::string(buf) + "\n" +
           "face = " + (face?"true":"false");
}

const Texture& Object::getAbsorb()const {
    if (!absorb) {
        std::cout<<"The absorb not defined yet."<<std::endl;
    }
    return *absorb;
}


Light::Light() : Object() {
}
Light::~Light() {
}

void Light::accept(const Json::Value& val) {
    brightness = val["brightness"].asDouble();
    Object::accept(val);
}
Color Light::getColor(const Vector&)const {
    if (texture->getType() != TEXTURE_PURE_COLOR)
        std::cout<<"The getColor of Light only support PURE_COLOR_MODE"<<std::endl;
    return texture->getColor();
}

Color::Color() {
    r = g = b = 0;
}

Color::Color(double r,double g,double b) : r(r),g(g),b(b) {
}

Color Color::adjust()const {
    return Color(min(r,1.0),min(g,1.0),min(b,1.0));
}


Color operator +(const Color& a,const Color &b) {
    return Color(a.r+b.r,a.g+b.g,a.b+b.b);
}

Color operator *(const Color& a,const Color &b) {
    return Color(a.r*b.r,a.g*b.g,a.b*b.b);
}

Color operator *(const Color& a,double k) {
    return Color(a.r*k,a.g*k,a.b*k);
}

Color Color::exp()const {
    return Color(::exp(r),::exp(g),::exp(b));
}

Color Color::operator +=(const Color& a) {
    r += a.r;
    g += a.g;
    b += a.b;
    return (*this);
}
void Color::accept(const Json::Value& val) {
    r = val["r"].asDouble();
    g = val["g"].asDouble();
    b = val["b"].asDouble();
}
std::string Color::description()const {
    std::stringstream ss;
    ss<<"Color<"<<r<<","<<g<<","<<b<<">"<<std::endl;
    std::string res;
    ss>>res;
    return res;
}



Camera::Camera(int rx,int ry):rx(rx),ry(ry) {
}
Camera::~Camera() {
}

void Camera :: accept(const Json::Value& val) {
    position.accept(val["position"]);
    dx.accept(val["dx"]);
    dy.accept(val["dy"]);
    origin.accept(val["origin"]);
    fdepth = val["fdepth"].asDouble();
}

void Camera::getRay(double scanX,double scanY, Vector& rayO, Vector& rayD) {
    Vector _rayO,_rayD;
    _rayO = origin;
    _rayD = ((position + scanX*dx/(rx-1) + scanY*dy/(ry-1))-origin).unit();
    addDepth(_rayO,_rayD,rayO,rayD);
}

void Camera::addDepth(const Vector& rayO,const Vector& rayD,Vector& resO,Vector& resD) {
    Vector FocalPlanePoint = rayO + rayD*fdepth;
    resO =	rayO + Vector::randomVectorOnSphere()*fdepth;
    resD = (FocalPlanePoint - rayO).unit();
}
