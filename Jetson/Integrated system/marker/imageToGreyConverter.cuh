//
// Created by Wihan on 2024-09-16.
//

#ifndef IMAGETOGREYCONVERTER_CUH
#define IMAGETOGREYCONVERTER_CUH


class imageToGreyConverter {
public:
    imageToGreyConverter(float rFactor, float gFactor, float bFactor, const int width, const int height);

    // Deallocate gpu memory if needed here
    ~imageToGreyConverter();

    float *imageToGrey(const float *inputImg) const;


private:
    float rFactor;
    float gFactor;
    float bFactor;

    int width;
    int height;

    // Scratch space pointer on gpu
    float *outputImg;
};


#endif //IMAGETOGREYCONVERTER_CUH
