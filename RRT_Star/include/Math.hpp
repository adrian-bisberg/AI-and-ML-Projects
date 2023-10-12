/// ---------
/// Portions of this file provided by author Mike Shah
//  from his class 5350 Computational Geometry at Northeastern
//  lines (14 - 107 and the associated implementations in math.cpp)
/// ---------

#ifndef MATH_HPP
#define MATH_HPP

#include <cmath> // for sqrt
#include <cassert>
#include <vector>

/*
 * Represent a two dimensional float point or vetor
 */
struct Vector2f{
    float x,y;

    /// Default constructor
    /// Initializes vector values to 0.0f by default.
    Vector2f(){
        x=0.0f;
        y=0.0f;
    }

    /// Two argument constructor
    Vector2f(float _x, float _y): x(_x), y(_y){
    }

    /// Copy Constructor
    /// Not truly needed, but in general I like to implement
    /// so I can debug when copies are made
    /// i.e. Add a 'print message' (or better yet -- see in the debugger)
    ///      when this is being called.
    Vector2f(const Vector2f& rhs){
        x = rhs.x;
        y = rhs.y;
    }

    /// Copy assignment operator
    Vector2f& operator=(const Vector2f& rhs){
        if(this== &rhs){
            return *this;
        }
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    /// Urnary Negation operator flips sign of vector.
    /// This produces a new vector.
    Vector2f operator-() const{
        return Vector2f(-x,-y);
    }

    Vector2f operator+(const Vector2f& rhs) const{
        return Vector2f(x+rhs.x, y+rhs.y);
    }

    // Subtraction
    Vector2f operator-(const Vector2f& rhs) const{
        return Vector2f(x-rhs.x, y-rhs.y);
    }

    // Subtract a vector from this one
    Vector2f& operator-=(const Vector2f& rhs){
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    // Normalize
    Vector2f& Normalize(){
        float len = Magnitude();
        assert(len != 0.0f && "We actually found a float that is 0 (or maybe close), uh oh!");
        x = x / len;
        y = y / len;

        // Return a reference so we can chain together calls
        return *this;
    }

    // Magnitude or "Length"
    float Magnitude(){
        return sqrt(x*x + y*y);	
    }

};


// Helpful class for storing the orientation between points.
enum class ORIENTATION{COLLINEAR, CLOCKWISE, COUNTERCLOCKWISE};

/// Produce a new normalized vector
/// NOTE: Since this produces a new vector,
///       the naming is 'NormalizedRetrieve'.
Vector2f NormalizedRetrieve(Vector2f in);

/// Compute the dot product of two vectors.
float Dot(const Vector2f& a, const Vector2f& b);

// Compute the cross product of two vectors.
float Cross(const Vector2f& a, const Vector2f& b);

// Return the midpoint between two points.
Vector2f CreateMidpoint(const Vector2f& a, const Vector2f& b);

// Determine if a point is on the left hand size of a line segment.
int isLeft(const Vector2f& a, const Vector2f& b, const Vector2f& point);

// Compute the distance between two points.
float Distance(const Vector2f& a, const Vector2f& b);

// Determine the orientation between a triplet of points directed from 
// p to q to r
ORIENTATION GetOrientation(Vector2f p, Vector2f q, Vector2f r);

/// @brief Determine if a point lies on a line segment.
/// @param a First end point of segment. 
/// @param b Secomd end point of segment.
/// @param p Point to test if on segment.
/// @return True if point on segment.
bool PointLiesOnSegment(const Vector2f& a, const Vector2f& b, const Vector2f& p);

/// @brief Determine if two segments intersect one another.
/// @param a1 First end point of segment one.
/// @param b1 First end point of segment two.
/// @param a2 Second end point of segment one.
/// @param b2 Second end point of segment two.
/// @return True if the segmets intersect one another.
bool SegmentsIntersect(const Vector2f& a1, const Vector2f& b1, const Vector2f& a2, const Vector2f& b2);

/// @brief Test if the line segment intersects or lies within a triangle.
/// @param a First end point of segment.
/// @param b Second end point of segment.
/// @param triangle The triangle to test.
/// @return True if the segment lies within the triangle at all.
bool SegmentInTriangle(const Vector2f& a, const Vector2f& b, std::vector<Vector2f>& triangle);


/// @brief Test if a point is inside a triangle.
/// @param v Point to test.
/// @param a First vertex of triangle.
/// @param b Second vertex of triangle.
/// @param c Thrid vertex of triangle.
/// @return 
bool PointInTriangle(const Vector2f& v, const Vector2f& a, const Vector2f& b, const Vector2f& c);

#endif
