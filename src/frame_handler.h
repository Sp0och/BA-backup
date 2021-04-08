#pragma once
#include "parameters.h"
#include "Frame.h"

struct frame_handler
{
public:

    // DBoW3::Database db;
    // DBoW3::Vocabulary* voc;

    map<int, cv::Mat> image_pool;

    list<Frame*> Framelist;
    
    frame_handler();
    // void loadVocabulary(std::string voc_path);
    
    void addFrame(Frame* cur_kf, bool flag_detect_loop);
    void addFrameIntoVoc(Frame* Frame);
    Frame* getFrame(int index);

    // int detectLoop(Frame* Frame, int frame_index);
};