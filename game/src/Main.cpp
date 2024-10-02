#include "Application.h"

int main(int argc, char *[])
{
    Application app;

    app.Setup();

    while (Application::IsRunning()) 
    {
        app.ProcessInput();
        app.Update();
        app.Render();
    }

    Application::Destroy();

    return 0;
}