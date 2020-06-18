//
// Created by 李曌珩 on 2019-06-23.
//

#ifndef PAINT_BOARD_H
#define PAINT_BOARD_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "core.h"
using namespace std;

class PaintBoard {
	private:
		cv::Mat *image;
		Color **board;
		int sizX,sizY;
	public:
		PaintBoard();
		~PaintBoard();
		void init(int sizX,int sizY,Color** _board);
		void update();
		void display();
		void save();
		void save_raw();
};

#endif
