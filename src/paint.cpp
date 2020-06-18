//
// Created by 李曌珩 on 2019-06-23.
//

#include "paint.h"
#include <iostream>
using namespace cv;

PaintBoard::PaintBoard() {
	sizX = -1,sizY = -1;
	image = NULL;
}
PaintBoard::~PaintBoard() {
}

void PaintBoard::init(int sizX,int sizY,Color** _board) {
	this->sizX = sizX;
	this->sizY = sizY;
	image = new Mat(this->sizX,this->sizY,CV_64FC3,Scalar(1.0,1.0,1.0));
	board = _board;
}

void PaintBoard::update() {
	assert(image);
	for (int i=0;i<sizX;i++) {
		for (int j=0;j<sizY;j++) {
			//Color& c = (*board)[(sizX-i-1)*sizY+j];
			Color& c = (*board)[i*sizY+j];
			Vec3f v(c.getR(),c.getG(),c.getB());
			image->at<Vec3d>(i,j) = v;
		}
	}
}
void PaintBoard::display() {
	imshow("picture", *image);
	waitKey(0);
}
void PaintBoard::save_raw() { 
    FILE* fout = fopen("result.ppm","w");
    fprintf(fout,"P3 %d %d\n",sizY,sizX);
    fprintf(fout,"255\n");
    for (int i=0;i<sizX;i++)
    {
        for (int j=0;j<sizY;j++)
        {
			Color c = (*board)[i*sizY+j]*255;
            fprintf(fout,"% 3d % 3d % 3d",int(c.getR()),int(c.getG()),int(c.getB()));
        }
        fprintf(fout,"\n");
    }
    fclose(fout);
}
void PaintBoard::save() {
    vector<int>compression_params;
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    imwrite("result.bmp", (*image)*255);
}
