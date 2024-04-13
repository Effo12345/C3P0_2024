#pragma once

#include <math.h>
#include <string>

namespace starlib {

/**
 * Store 2D point data and define 2D operations
*/
struct Point {
    float x;
    float y;

    //Overload math operators to interact with other point objects and with
    //scalar values
    Point operator+(const Point& p) {
        return {x + p.x, y + p.y};
    } 

    Point operator+(const float f) {
        return {x + f, y + f};
    }

    Point operator-(const Point& p) {
        return {x - p.x, y - p.y};
    } 

    Point operator-(const float f) {
        return {x - f, y - f};
    }

    Point operator*(const float f) {
        return {x * f, y * f};
    }

    Point operator/(const float f) {
        return {x / f, y / f};
    }


    void operator+=(const Point& p) {
        x += p.x;
        y += p.y;
    }

    void operator-=(const float f) {
        x += f;
        y += f;
    }


    /**
     * Treats the x and y values as the components of a vector and returns
     * its magnitude using the pythagorean theorem
     */
    float getMagnitude() {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    /**
     * Returns the distance between the current point and another point 
     * using the distance formula
     */
    float distanceTo(Point p) {
        return sqrt(pow((x - p.x), 2) + pow((y - p.y), 2));
    }

    /**
     * Returns the dot product of this point and the parameter p
     */
    float dotProduct(Point p) {
        return (x * p.x) + (y * p.y);
    }

    /**
     * Returns the point value as a string for telemetry
     */
        std::string toString() {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
        }
	};

} // namespace starlib