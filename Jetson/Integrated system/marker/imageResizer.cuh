//
// Created by Wihan on 2024-09-24.
//

#ifndef IMAGERESIZER_CUH
#define IMAGERESIZER_CUH



class imageResizer {
public:
    imageResizer(int w, int h);
    ~imageResizer();

    float* Resize(const int inputW, const int inputH, const float *data, const int channels) const;
private:
    int w, h;

    // Scratch space pointer on gpu
    float *outputImg;
};

#endif //IMAGERESIZER_CUH
