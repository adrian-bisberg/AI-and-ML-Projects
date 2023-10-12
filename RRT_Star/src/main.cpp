// C++ Standard Libraries
#include <iostream>
// Third Party
#if defined(LINUX) || defined(MINGW)
    #include <SDL2/SDL.h>
#else // This works for Mac
    #include <SDL.h>
#endif

#include <fstream>
#include <vector>
#include <numeric> // for iota
#include <algorithm>
#include <cstdlib>

#include "Math.hpp"
#include "RRT.hpp"

#include <chrono>
using namespace std::chrono;

// Entry point to program
int main(int argc, char* argv[]){

    // Define our values to use for RRTStar initialization.
    Vector2f start = {10, 10};
    Vector2f goal = {580, 460};
    int goalRadius = 20;
    int neigbordoodRadius = 50;
    int rho = 30;
    int maxIterations = 3000;

    // Ensure that a file with obstacles has been provided
    if(argc < 2){
        std::cout << "Provide at least 1 argument for text file containing obstacle polygons." << std::endl;
        std::cout << "e.g. ./prog points.txt" << std::endl;
        std::cout << "Alternately provide additonal arguments for start and goal to override defaults." << std::endl;
        std::cout << "e.g. ./prog points.txt <start_position_x> <start_position_y> <goal_position_x> <goal_position_y <goal_radius>" << std::endl;
        return 0;
    }

    if(argc > 2){
        if (argc != 7){
            std::cout << "Provide at least 1 argument for text file containing obstacle polygons." << std::endl;
            std::cout << "e.g. ./prog points.txt" << std::endl;
            std::cout << "Alternately provide additonal arguments for start and goal to override defaults. Must provide all together." << std::endl;
            std::cout << "e.g. ./prog points.txt <start_position_x> <start_position_y> <goal_position_x> <goal_position_y <goal_radius>" << std::endl;
            return 0;
        }
        start = {(float)atoi(argv[2]), (float)atoi(argv[3])};
        goal = {(float)atoi(argv[4]), (float)atoi(argv[5])};
        goalRadius = atoi(argv[6]);
    }

    // Initialize the obstacles with provided input file
    Obstacles obs = Obstacles(argv[1]);

    RRTStar rrt = RRTStar(640, 480, obs, start, goal, goalRadius, 70, 30);

    rrt.findBestPath();
    std::cout << "Path found of length: " << rrt.getCost() << std::endl;

    // SDL initialization
    if(SDL_Init(SDL_INIT_VIDEO) < 0){
        std::cout << "SDL could not be initialized: " <<
                  SDL_GetError();
    }else{
        std::cout << "SDL video system is ready to go\n";
    }

    // Create SDL window
    SDL_Window* window=nullptr;
    window = SDL_CreateWindow("C++ SDL2 Window",20, 20, 640, 480,SDL_WINDOW_SHOWN);

    // Create SDL renderer
    SDL_Renderer* renderer = nullptr;
    renderer = SDL_CreateRenderer(window,-1,SDL_RENDERER_ACCELERATED);

    // Allows for alpha values 
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    bool visualization = true;

    // Loop to draw visualization
    while(visualization){
        SDL_Event event;

        // Quit the visualization
        while(SDL_PollEvent(&event)){
            if(event.type == SDL_QUIT){
                visualization= false;
            }
        }
        
        // Set the backgorund of drawing
        SDL_SetRenderDrawColor(renderer,0,0,0,SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);

        // Draw the set of obstacles
        SDL_SetRenderDrawColor(renderer,152,115,172,SDL_ALPHA_OPAQUE);
        obs.draw(renderer);

        // Draw the tree generated
        rrt.drawTree(renderer);

        //Draw the retrieved path
        rrt.drawPath(renderer);

        // Draw the goal region for reference.
        drawDottedCircle(renderer, goal, goalRadius);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
