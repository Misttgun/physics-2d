#include "Application.h"

int main(int argc, char *[])
{
    Application app;

    app.Setup();

    while (app.IsRunning()) 
    {
        app.ProcessInput();
        app.Update();
        app.Render();
    }

    app.Destroy();

    return 0;
}