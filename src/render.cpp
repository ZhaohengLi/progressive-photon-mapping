//
// Created by 李曌珩 on 2019-06-23.
//

#include "render.h"
#include "paint.h"
#include <iostream>

Render::Render(){
}

Render::~Render() {
}

void Render::accept(const Json::Value& val) {
	rx = val["rx"].asInt();
	ry = val["ry"].asInt();
	std::cout<<"Reander created <"<<rx<<","<<ry<<">"<<std::endl;
}

void Render::update() {
	this->paint_board->update();
}

void Render::registerPaintBoard(PaintBoard* pb) {
	paint_board = pb;
	paint_board->init(rx,ry,&board);
}
