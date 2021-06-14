#include <stdlib.h>
#include "ros/ros.h"
#include <SDL2/SDL.h>
#include <iostream>
#include "geometry_msgs/Vector3.h"
//#include "keyboard/customMsg.h"
#include "pubsub/keystateMsg.h"

const int DIRECTION_SIZE = 6;

enum ControlScheme {
    STANDARD = 1,
    INVERTED = -1
};

class KeystateController {

public:
    ~KeystateController();
    KeystateController(ControlScheme scheme);

    //Checking keyboard state:
    bool checkState();

    //For publishing
    void publish();

    //For saving control information
    int direction[DIRECTION_SIZE];

private:
    //Fro holding keyboard state
    const Uint8 *state;

    //For publishing information with ROS
    ros::NodeHandle n;
    ros::Publisher p;

    //Storing current control scheme
    ControlScheme scheme;

    //Event handler
    //SDL_Event e;

    SDL_Window * window;
    SDL_Renderer * renderer;

    SDL_Surface * image;
    SDL_Texture * texture;
};

//Constructor
KeystateController::KeystateController(ControlScheme scheme) {
    //Intializing publisher:
    //p = n.advertise<geometry_msgs::Vector3>("direction", 1);
    p = n.advertise<pubsub::keystateMsg>("direction", 1);
    this->scheme = scheme;

    //Initialize SDL
    SDL_Init(SDL_INIT_VIDEO);

    //Retrieving new keyboard state
    //SDL_PumpEvents();
    state = SDL_GetKeyboardState(NULL);

    //Setup SDL window, texture and surface
    window = SDL_CreateWindow("SDL2 Keyboard/Mouse events", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 200, 200, 0);
    renderer = SDL_CreateRenderer(window, -1, 0);
    image = SDL_LoadBMP("/home/smed/test/src/keyboard/src/joystick.bmp");
    texture = SDL_CreateTextureFromSurface(renderer, image);
    SDL_FreeSurface(image);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    //render
    //Creating rectangle x0 y0 x1   y1
    SDL_Rect dstrect = { 0, 0, 200, 200 };
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, &dstrect);
    SDL_RenderPresent(renderer);
}

//Destructor
KeystateController::~KeystateController() {

    //Release resources to prevent leeks
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    //Closing sdl;
    SDL_Quit();
}

//function that saves the keystate, and returns true if any key is pressed
bool KeystateController::checkState() {
    //std::cout << "Checking state" << std::endl;

    bool keyPressed = false;

    //Requesting update for keyboard state
    //SDL_PollEvent(&e);
    SDL_PumpEvents();

    //resetting values of direction
    for (int i = 0; i < DIRECTION_SIZE; i++) {
        direction[i] = 0;
    }

    //checking for arrow keys
    if (state[SDL_SCANCODE_A]) {
        direction[1] = 1;
        std::cout << "left" << std::endl;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_D]) {
        direction[1] = -1;
        std::cout << "right" << std::endl;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_W]) {
        direction[2] = 1;
        std::cout << "forward" << std::endl;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_S]) {
        direction[2] = -1;
        std::cout << "backwards" << std::endl;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_Q]) {
        direction[0] = 1;
        std::cout << "up" << std::endl;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_E]) {
        direction[0] = -1;
        std::cout << "Down" << std::endl;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_R]) {
        direction[3] = 1;
        std::cout << "Grap" << std::endl;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_T]) {
        direction[3] = -1;
        std::cout << "Release" << std::endl;
        keyPressed = true;
    }
    
    if (state[SDL_SCANCODE_F]) {
        direction[4] = 1;
        std::cout << "auto" << std::endl;
        keyPressed = true;
    }
    
    if (state[SDL_SCANCODE_G]) {
        direction[4] = -1;
        std::cout << "Home" << std::endl;
        keyPressed = true;
    }

    if (state[SDL_SCANCODE_Z]) {
        direction[5] = -1;
        std::cout << "state move" << std::endl;
        keyPressed = true;
    }
    
    if (state[SDL_SCANCODE_X]) {
        direction[5] = 1;
        std::cout << "state turn" << std::endl;
        keyPressed = true;
    }
    return keyPressed;
}

void KeystateController::publish() {
    //geometry_msgs::Vector3 msg;
    pubsub::keystateMsg msg;
    msg.x = direction[0];
    msg.y = direction[1];
    msg.z = direction[2];
    msg.r = direction[3];
    msg.a = direction[4];
    msg.s = direction[5];

    //std::cout << direction[2] << '\n';

    p.publish(msg);
}

ControlScheme getControlScheme() {
std::cout << "Hello" << std::endl;   
}


int main(int argc, char **argv) {

    //Initializing
    ros::init(argc, argv, "keystate");

    //Create instance of class
    KeystateController k(getControlScheme());

    //While ROS is running, loop forever
    //Loop 10 times per second
    std::cout << "Commencing Loop" << std::endl;
    ros::Rate loopRate(100);
    while (ros::ok()) {

        //Check for keystate
        if (k.checkState()) {
            //std::cout << "Keypress detected" << std::endl;
        }

        k.publish();

        //Handle publishing etc..
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
