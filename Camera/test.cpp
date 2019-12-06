#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <typeinfo>
#include "../Camera/camera.h"


Mycamera mycamera;
int main()
{
        mycamera.openWorkThread();
        while (1)
        {
                sleep(1);
                std::cout<<"main thread"<<std::endl;
                if(mycamera.endmain_flag == 1)
                {
                        break;
                }
        }
}