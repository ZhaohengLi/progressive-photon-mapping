//
// Created by 李曌珩 on 2019-06-23.
//
#include <iostream>
#include "paint.h"
#include "json/json.h"
#include "ppm.h"
#include <unistd.h>
#include <fstream>
#include <pthread.h>

int main(int argc, char** args)
{
    std::ifstream ifs("configures/my.json");
    Json::CharReaderBuilder reader;
    Json::Value root;
    JSONCPP_STRING errs;
    Json::parseFromStream(reader, ifs, &root, &errs);

    Render *PPM = new ProgressivePhotonMapping();

    PaintBoard PB;
    PPM->accept(root);
    PPM->registerPaintBoard(&PB);
    PPM->run();
    PB.update();
    PB.save();
}
