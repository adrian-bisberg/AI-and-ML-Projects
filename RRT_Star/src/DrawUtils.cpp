#include "DrawUtils.hpp"

#include <cmath>


// Helper funciton to draw a circle at center with radius, done simply by
// drawing dotted points around the circle for ease of implementation
void drawDottedCircle(SDL_Renderer* renderer,  Vector2f& center, int radius){
    SDL_SetRenderDrawColor(renderer,200,30,30,SDL_ALPHA_OPAQUE);
    for(float theta = 0; theta < (M_PI * 2); theta += ((M_PI * 2) / 40)){
        float x = center.x + radius * cos(theta);
        float y = center.y + radius * sin(theta);
        SDL_RenderDrawPoint(renderer,x,y);
    }
}

// Draw a thicker point
void DrawPointScaled(SDL_Renderer* renderer, int x, int y, size_t size){
    for(int s = x; s < x+size; ++s){
        for(int t = y; t < y+size; ++t){
            SDL_RenderDrawPoint(renderer,s,t);
        }
    }
}