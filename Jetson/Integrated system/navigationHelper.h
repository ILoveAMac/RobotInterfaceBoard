#ifndef NAVIGATIONHELPER_H
#define NAVIGATIONHELPER_H

#include <vector>
#include <math.h>
#include <iostream>

#include "serialHelper.h"
#include "positionController.h"
#include "pid.h"

class navigationHelper
{
public:
    navigationHelper();
    ~navigationHelper();

private:
    serialHelper serial;
    positionController controller;
};

#endif // NAVIGATIONHELPER_H