#ifndef VECTOR2D
#define VECTOR2D

#include <iostream>
namespace HybridAStar {
//###################################################
//                                            VECTOR2
//###################################################
/// A class describing a simple 2D vector
    class Vector2D {
    public:
        /// default constructor
        inline Vector2D(const double x = 0, const double y = 0) { this->x = x; this->y = y; }
        /// a method to multiply a vector by a scalar
        inline Vector2D operator * (const double k) const { return Vector2D(x * k, y * k); }
        /// a method to divide a vector by a scalar
        inline Vector2D operator / (const double k) const { return Vector2D(x / k, y / k); }
        /// a method to add a vector to a vector
        inline Vector2D operator + (const Vector2D& b) const { return Vector2D(x + b.x, y + b.y); }
        /// a method to subtract a vector from a vector
        inline Vector2D operator - (const Vector2D& b) const { return Vector2D(x - b.x, y - b.y); }
        /// a method to negate a vector
        inline Vector2D operator - () const  {return Vector2D(-x, -y);}
        /// a convenience method to print a vector
        friend std::ostream& operator<<(std::ostream& os, const Vector2D& b) {os << "(" << b.x << "|" << b.y << ")"; return os; }
        /// a method to calculate the length of the vector
        double length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); }
        /// a method to calculate the length of the vector
        double sqlength() const { return x*x + y*y; }
        /// a method to calculate the dot product of two vectors
        double dot(Vector2D b) { return x * b.x + y * b.y; }
        ///a method that returns the orthogonal complement of two vectors
        inline Vector2D ort(Vector2D b) {
            Vector2D a(this->x, this->y);
            Vector2D c;
            // multiply b by the dot product of this and b then divide it by b's length
            c = a - b * a.dot(b) / b.sqlength();
            return c;
        }
        inline double getX() { return x; }
        inline double getY() { return y; }

        void setX(const double xx) {this->x = xx;}
        void setY(const double yy) {this->y = yy;}

        //  void setT(double t) { this->t = t; }
        //  double getT() { return t; }
    private:
        /// the x part of the vector
        double x;
        /// the y part of the vector
        double y;
        //  /// the theta part for plotting purposes
        //  double t;
    };
    inline Vector2D operator * (double k, const Vector2D& b) {
        return (b * k);
    }
}
#endif // VECTOR2D
