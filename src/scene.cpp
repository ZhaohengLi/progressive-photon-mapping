//
// Created by 李曌珩 on 2019-06-23.
//


#include "scene.h"
#include "core.h"
#include "object.h"


Scene::Scene() {
	camera = NULL;
}

Scene::~Scene() {
	if (camera)
		delete camera;
}

void Scene::accept(const Json::Value& val,int _rx,int _ry) {
	if (val["camera"]["type"].asString() == "default") {
		camera = new Camera(_rx,_ry);
		camera->accept(val["camera"]);
	}

	for (int i=0;i<val["objects"].size();i++) {
		Json::Value v = val["objects"][i];
		std::string tag = v["type"].asString();
		if (tag == "sphere") {
			objects.push_back(new Sphere());
			objects.back()->accept(v);
		}else if (tag == "plane") {
			objects.push_back(new Plane());
			objects.back()->accept(v);
		}else if (tag == "bazier_curves") {
			int n = v["ctrl_pts"].size();
			for (int i=0;i<n;i++) {
				//if (i!=0 && i!=5)continue;
				objects.push_back(new BazierCurve(i));
				objects.back()->accept(v);
			}
		}else if(tag[0] == '#') {
		}else {
		}
	}

	for (int i=0;i<val["lights"].size();i++) {
		Json::Value v = val["lights"][i];
		std::string tag = v["type"].asString();

		if (tag == "area_light") {
			lights.push_back(new AreaLight());
			lights.back()->accept(v);
		}else if (tag == "point_light") {
			lights.push_back(new PointLight());
			lights.back()->accept(v);
		}else if (tag[0] == '#') {

		}else {
		}
	}
	bg_color.accept(val["bg_color"]);
}

const Object* Scene::findCollidedObject(const Vector& _rayO,const Vector& _rayD,Collision& resColl) {
	Vector rayO = _rayO;
	Vector rayD = _rayD;
	Object* ret = NULL;
	resColl.dist = inf;
	for (auto obj : objects) {
		Collision obj_coll;
		if (obj->collideWith(rayO,rayD,obj_coll)) {
			if (obj_coll.dist < resColl.dist) {
				ret = obj;
				resColl = obj_coll;
			}
		}
	}
	return ret;
}

const Light* Scene::findCollidedLight(const Vector& _rayO, const Vector& _rayD,Collision& resColl) {
	Vector rayO = _rayO;
	Vector rayD = _rayD;
	Light* ret = NULL;
	resColl.dist = inf;
	for (auto &lgt : lights) {
		Collision lgt_coll;
		if (lgt->collideWith(rayO,rayD,lgt_coll)) {
			if (lgt_coll.dist < resColl.dist) {
				resColl = lgt_coll;
				ret = lgt;
			}
		}
	}
	return ret;
}

