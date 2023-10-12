#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "Math.hpp"
#include <vector>
#include <string>

using Triangle = std::vector<Vector2f>;
using Vertex = Vector2f;

// Custom exception, contains functions will only be supported
// once a polygon has been triangulated.
class UnTriangulatedPolygon : public std::exception {
    public:
const char * what () {
    std::string message = "Does not support contains functionality for polygons that have not been triangulated. Please call TriangulateEarClipping first.";
    return message.c_str();
}
};

struct Polygon{

    Polygon(std::vector<Vertex> vertices);

    std::vector<Vertex> vertices; //< List of points at each vertex of the polygon.
    std::vector<int> vertexIndices; //< List of 0 based indexing of each vertex.
    std::vector<Triangle> triangles; //< List of triangles if polygon is triangulated. Empty if not.

    /// @brief Use the ear clipping algorithm to triangulate a polygon. Assumes that
    ///        the prodvided polygon: contains no intersecting lines, colinear vertices
    ///        or cut out holes in the polygon. Stores the results in the triangles data member. 
    void TriangulateEarClipping();

    /// @brief Test if the polygon contains some point.
    /// @param point The point to test.
    /// @return True if the point is inside or on the polygon.
    bool contains(const Vector2f& point);

    /// @brief Test if a line segment is contained in the polygon.
    /// @param a The first end point of the line segment.
    /// @param b The second end point of the line segment.
    /// @return True if the segment is at all contained in or on the polygon.
    bool containsSegment(const Vector2f& a, const Vector2f& b);
};

/// Helper function that will 'wrap-around' a data structure given a 
/// provided index.
template <typename T>
int GetIndex(const std::vector<T> v, int index){
    // Retrieve the size of our vector
    int len = v.size();
    // Most of the time, we'd hope to be within the bounds
    // of our data structure, so a simple test.
    if(index < len && index > 0){
        return index;
    }

    // If our index is greater than the size, we need to wrap around
    // Most often this happens when accessing the (last_element + 1)
    if(index >= len){
        return index % len;
    }

    // If the index is smaller than 0, then we need to wrap to the
    // last entry.
    // Note: A negative value % len returns a negative value.
    //       We then add the length of our collection, to wrap us
    //       back up to a positive value within the collection.
    if(index < 0){
        return index % len + len; 
    }

    // Default case
    return index;
}

#endif