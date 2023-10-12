#if defined(LINUX) || defined(MINGW)
    #include <SDL2/SDL.h>
#else // This works for Mac
    #include <SDL.h>
#endif

#include "Math.hpp"

// Helper funciton to draw a circle at center with radius, done simply by
// drawing dotted points around the circle for ease of implementation
void drawDottedCircle(SDL_Renderer* renderer,  Vector2f& center, int radius);

// Draw a thicker point
void DrawPointScaled(SDL_Renderer* renderer, int x, int y, size_t size=2);

void DrawThickLine(SDL_Renderer* renderer, int x, int y);