#include "control.h"

control::control(/* args */)
{
    
}

control::~control()
{
}

void control::run(){
    
    while (true){
        actuatorControl.run();
    }
    

}

int main(){
    control controlobj;
    controlobj.run();

}