#include "ImageProgress.h"
#include <thread>

int main()
{    
    ImageProgress main_progress;

    thread produce(&ImageProgress::ImageProducer,&main_progress);
    thread consume(&ImageProgress::ImageConsumer2, &main_progress);
    thread fitting(&ImageProgress::Fitting, &main_progress);
    thread readport(&ImageProgress::ReadIMU2, &main_progress);
    thread sendport(&ImageProgress::SendData2, &main_progress);
    thread savevideo(&ImageProgress::VideoSave, &main_progress);

    produce.join();
    consume.join();
    fitting.join();
    readport.join();
    sendport.join();
    savevideo.join();
}
