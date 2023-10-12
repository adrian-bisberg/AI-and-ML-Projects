#include <string>
#include <vector>
#include <fstream>

#if defined(LINUX) || defined(MINGW)
    #include <SDL2/SDL.h>
#else // This works for Mac
    #include <SDL.h>
#endif

#include "Math.hpp"
#include "Polygon.hpp"

class Obstacles{

public:

    Obstacles(std::string filename){
        std::ifstream file;

        file.open(filename);

        if (!file.is_open()){
            throw std::invalid_argument("Unable to open file.");
        }

        std::string line;
        std::vector<Vector2f> temp;
        while(getline(file, line)){
            if (line.size() == 0){
                m_polygons.push_back(Polygon(temp));
                m_polygons.at(m_polygons.size()-1).TriangulateEarClipping();
                temp.clear();
            }else{
                temp.push_back(parseVector(line));
            }
        }
        m_polygons.push_back(Polygon(temp));
        m_polygons.at(m_polygons.size()-1).TriangulateEarClipping();
    
        file.close();
    };

    void draw(SDL_Renderer* renderer){
        for(Polygon polygon: m_polygons){
            drawPolygon(renderer, polygon.vertices);
        }
    }

    bool inObstacles(const Vector2f& point){
        for(Polygon polygon : m_polygons){
            if (polygon.contains(point)){
                return true;
            }
        }
        return false;
    }

    bool segmentInObstacles(const Vector2f& a, const Vector2f&b){
        for(Polygon polygon : m_polygons){
            if (polygon.containsSegment(a,b)){
                return true;
            }
        }
        return false;
    }

private:
    std::vector<Polygon> m_polygons;
    
    // Get a vector representation from a string listing x y coordinates seperated by a space.
    Vector2f parseVector(const std::string s) {
        int start = 0;
        int end = 0;
        std::string token;
        std::vector<std::string> values;

        while ((end = s.find(" ", start)) != std::string::npos) {
            token = s.substr(start, end - start);
            start = end + 1;
            values.push_back (token);
        }

        values.push_back(s.substr(start));
    
        if(values.size() != 2){
            throw std::invalid_argument("Malformed point definition. For each vertex specify as 'x y' on a single line.");
        }

        return {stof(values.at(0)), stof(values.at(1))};
    }
    
    void drawPolygon(SDL_Renderer* renderer, const std::vector<Vector2f>& points){
        for(size_t i = 0; i < points.size(); i++){
            if(i == points.size() - 1){
                SDL_RenderDrawLine(renderer, points.at(i).x, points.at(i).y, points.at(0).x, points.at(0).y);
            }else{
                SDL_RenderDrawLine(renderer, points.at(i).x, points.at(i).y, points.at(i + 1).x, points.at(i + 1).y);
            }
        }
    }

};