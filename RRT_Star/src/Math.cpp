#include "Math.hpp"

#include <algorithm>
#include <cmath>

Vector2f NormalizedRetrieve(Vector2f in)
{
			float len = in.Magnitude();
			in.x = in.x / len;
			in.y = in.y / len;

			return Vector2f(in.x,in.y);
}

float Dot(const Vector2f& a, const Vector2f& b)
{
    return (a.x * b.x) + (a.y * b.y);
}

float Cross(const Vector2f& a, const Vector2f& b)
{
    float result = (a.x * b.y) - (a.y * b.x);
    return result; 
}

Vector2f CreateMidpoint(const Vector2f& a, const Vector2f& b)
{
	return Vector2f((a.x+b.x)/2,(a.y+b.y)/2);
}

int isLeft(const Vector2f& a, const Vector2f& b, const Vector2f& point)
{
    return ( (b.x - a.x) * (point.y - a.y) - (point.x - a.x) * (b.y - a.y) );
}

float Distance(const Vector2f& a, const Vector2f& b)
{
	return std::sqrt(((b.x - a.x) * (b.x - a.x)) + (b.y - a.y)* (b.y - a.y));
}


ORIENTATION GetOrientation(Vector2f p, Vector2f q, Vector2f r)
{
    Vector2f pq = q - p;
    Vector2f qr = r - q;

    int crossProd = Cross(pq, qr);

    if(crossProd > 0){
        return ORIENTATION::COUNTERCLOCKWISE;
    }else if(crossProd < 0){
        return ORIENTATION::CLOCKWISE;
    }else{
        return ORIENTATION::COLLINEAR;
    }
};

bool PointLiesOnSegment(const Vector2f& a, const Vector2f& b, const Vector2f& p)
{
	if(b.x <= std::max(a.x, p.x) && b.x >= std::min(a.x, p.x)
	   && b.y <= std::max(a.y, p.y) && b.y >= std::min(a.y, p.y)){
		return true;
	}
	else{
		return false;
	}
}

bool PointInTriangle(const Vector2f& v, const Vector2f& a, const Vector2f& b, const Vector2f& c)
{
	int first  = isLeft(a,b,v);	
	int second = isLeft(b,c,v);	
	int third  = isLeft(c,a,v);	

	return (first>0 && second>0 && third>0);
}

bool SegmentsIntersect(const Vector2f& a1, const Vector2f& b1, const Vector2f& a2, const Vector2f& b2)
{

	ORIENTATION b1_a = GetOrientation(a1, b1, a2);
	ORIENTATION b_a1 = GetOrientation(a1, b1, b2);
	ORIENTATION b2_a = GetOrientation(a2, b2, a1);
	ORIENTATION b_a2 = GetOrientation(a2, b2, b1);

	if (b1_a != b_a1 && b2_a != b_a2){
		return true;
	}

	if (b1_a == ORIENTATION::COLLINEAR && PointLiesOnSegment(a1, a2, b1)){
		return true;
	}

	if (b_a1 == ORIENTATION::COLLINEAR && PointLiesOnSegment(a1, b2, b1)){
		return true;
	}

	if(b2_a == ORIENTATION::COLLINEAR && PointLiesOnSegment(a2, a1, b2)){
		return true;
	}

	if (b_a2 == ORIENTATION::COLLINEAR && PointLiesOnSegment(a2, b1, b2)){
		return true;
	}

	return false;
}


bool SegmentInTriangle(const Vector2f& a, const Vector2f& b, std::vector<Vector2f>& triangle)
{
   
   // Case 1: The segment intersects with one of the polygon edges.
   for(int i = 0; i < triangle.size(); i++){
	  if(i == triangle.size() - 1){
		if(SegmentsIntersect(a, b, triangle.at(i), triangle.at(0))){
			return true;
		}
	  }else if(SegmentsIntersect(a, b, triangle.at(i), triangle.at(i + 1))){
			return true;
	  }
   }

   // Case 2: Segment must lie entirely inside polygon
   // check any point and see if on the interior of the polygon
   if(PointInTriangle(a, triangle[0], triangle[1], triangle[2])){
	  return true;
   }

   // Otherwise segment must be entirely outside polygon.
   return false;

}