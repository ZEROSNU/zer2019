#include <iostream>
#include <string>
#include <map>
#include "Model.h"
#include "ModelHandler.h"


ModelHandler mh = ModelHandler("handler");



int main(void) {
    Model linear = ModelHandler("linear");

    std::cout << "cpp file" << std::endl;
    return 0;
}