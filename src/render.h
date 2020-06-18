//
// Created by 李曌珩 on 2019-06-23.
//


#ifndef RENDER_H
#define RENDER_H
#include "json/json.h"
#include "paint.h"
#include "core.h"

class Render {
	protected:
		PaintBoard* paint_board;
		Color *board;
		int rx,ry;
	public:
		Render();
		~Render();
		virtual void accept(const Json::Value &val) = 0;
		virtual void run() = 0;
		void registerPaintBoard(PaintBoard* pb) ;
		void update();
};

#endif
