#include "module.h"

#include <cstdio>

Module::Module()
{
    printf("\nCreating a std module\n");
}

Module::~Module(){}

void Module::runModule()
{
    this->update();
}


void Module::runModulePost()
{
    this->updatePost();
}

void Module::update(){}
void Module::updatePost(){}
void Module::configure(){}
void Module::handleInterrupt(){}
