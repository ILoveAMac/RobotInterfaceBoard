#include "navigationHelper.h"

navigationHelper::navigationHelper() : serial("/dev/ttyUSB0", 9600), controller(0.5, 10, 0.1, 0.05)
{
}

navigationHelper::~navigationHelper()
{
}
