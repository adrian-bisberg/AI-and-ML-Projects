#include "Polygon.hpp"
#include <iostream>

Polygon::Polygon(std::vector<Vertex> input_vertices) : vertices(input_vertices)
{
    for (int i = 0; i < vertices.size(); i++){
        vertexIndices.push_back(i);
    }
}

bool Polygon::contains(const Vector2f& point)
{
    if(triangles.empty()){
        throw UnTriangulatedPolygon();
    }
    for(Triangle t: triangles){
        if (PointInTriangle(point, t[0], t[1], t[2])){
            return true;
        }
    }
    return false;
}

bool Polygon::containsSegment(const Vector2f& a, const Vector2f& b)
{
    if(triangles.empty()){
        throw UnTriangulatedPolygon();
    }
    for(Triangle t: triangles){
        if (SegmentInTriangle(a, b, t)){
            return true;
        }
    }
    return false;
}

void Polygon::TriangulateEarClipping()
{

    // Copy the list of vertex indices so we can update it as we go
    // without changing origional list.
    std::vector<int> indices = vertexIndices;

    // Loop unil three indices (one triangle) remain.
    while(indices.size() > 3){
        // Move through each remaining index in order to find
        // a valid ear candiate.
        for(int i = 0; i < indices.size(); i++){

            // Get the index of the current vertex being checked, as well as previous and next vertex.
            int a = indices[i];
            int b = indices[GetIndex(indices, i - 1)];
            int c = indices[GetIndex(indices, i + 1)];

            Vertex check = vertices.at(a);
            Vertex prev = vertices.at(b);
            Vertex next = vertices.at(c);

            Vector2f check_to_next = next - check;
            Vector2f check_to_prev = prev - check;

            // If cross product < 0, angle at check vertex (moving from previous to the next)
            // is over 180 degrees, this vertex is not valid as an ear so continue to next.
            if(Cross(check_to_prev, check_to_next) < 0.0f){
                continue;
            }

            bool isEar = true;
        
            // Check each other vertices in the polygon to see if any point lies inside the triangle.
            for (int j = 0; j < vertices.size(); j++){
                // Don't check the points being used in the triangle.
                if (j != a && j != b && j != c){
                    if (PointInTriangle(vertices[j], check, prev, next)){
                        // If any other points are in the triangle, cannot be an ear.
                        isEar = false;
                        break;
                    }
                }
            }

            // Add the triangle and remove index if it is a valid ear.
            if (isEar){
                Triangle add{check, prev, next};
                triangles.push_back(add);
                indices.erase(indices.begin() + i);
                break;
            }
        }
    }

    // Add the final remaining triangle.
    triangles.push_back({vertices[indices[0]], vertices[indices[1]], vertices[indices[2]]});
}