#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP

#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath> 

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (abs(d1 - d2) < epsilon){
            return true;
        } else {
            return false;
        }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg/(180.0/PI);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad*(180.0/PI);
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief add two vectors together and update the value
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtract two vectors together and update the value
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Vector2D & operator-=(const Vector2D & rhs);


        /// \brief subtract two vectors together and update the value
        /// \param scalar - the first transform to apply
        /// \return a reference to the newly transformed operator
        Vector2D & operator*=(double scalar);
    };

    /// \brief add two vectors and return a new variable
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    Vector2D operator+(Vector2D & lhs, const Vector2D & rhs);

    /// \brief subtract two vectors and return a new variable
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    Vector2D operator-(Vector2D & lhs, const Vector2D & rhs);

    /// \brief subtract two vectors and return a new variable
    /// \param scalar - the first transform to apply
    /// \return the composition of the two transforms
    Vector2D operator*(Vector2D & rhs, double scalar);

    /// @brief multiply a vector 2D by a scalar
    /// @param scalar - scalar to multiply by
    /// @param rhs - vector to be scaled
    /// @return the scaled vector
    Vector2D operator*(double scalar, const Vector2D & rhs);

    /// \brief subtract two vectors and return a new variable
    /// \param rhs - the right hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    double dot(const Vector2D & lhs, const Vector2D & rhs);

    /// \brief subtract two vectors and return a new variable
    /// \param vec - the first transform to apply
    /// \return the composition of the two transforms
    double magnitude(Vector2D vec);

    /// \brief subtract two vectors and return a new variable
    /// \param vec - the first transform to apply
    /// \return the composition of the two transforms
    double angle(const Vector2D lhs, const Vector2D rhs);

    /// \brief multiply two transforms together, returning their composition
    /// \param scalar - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    Vector2D operator*(double scalar, const Vector2D & rhs);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);


    /// The way input works is (more or less): what the user types is stored in a buffer until the user types
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin) and processing stops.
    ///
    /// We have lower level control however.
    /// std::peek() looks at the next unprocessed character in the buffer without removing it
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// std::get() removes the next unprocessed character from the buffer.
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/get
    /// When you call std::peek() it will wait for there to be at least one character in the buffer (e.g., the user types a character)
    /// HINT: this function can be written in under 20 lines and uses only std::peek(), std::get(), istream::operator>>() and a little logic
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief a class to handle an object with a rotation and x, y coordinate
    class Twist2D
    {
    private:
        double angular;
        Vector2D translate;
    public:
        /// \brief Create an twist with no velocities
        Twist2D();

        /// \brief create a transformation that is a pure translational velocity
        /// \param trans - the x and y velocities
        explicit Twist2D(Vector2D trans);

        /// \brief create a pure rotational velocity
        /// \param radians - angular velocity in radians per second
        explicit Twist2D(double radians);

        /// \brief create a twist with angular and rotational vel
        /// \param ang - the angular velocity
        /// \param trans - the rotational velocity, in radians per second
        Twist2D(double ang, Vector2D trans);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Twist2D & tf);

        /// \brief getter for the angular velocity
        /// \return  angular
        double getAng(){
            return angular;
        }

        /// \brief getter for the translational velocity
        /// \return  translate
        Vector2D getTran(){
            return translate;
        }

        /// \brief \see class Transform2D (declared outside this class)
        friend class Transform2D;
    };

    /// \brief should print a human readable version of the Twist:
    /// An example output:
    /// ang: 90 vx: 3 vy: 5
    /// \param os - an output stream
    /// \param tf - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & tf);

    /// \brief Read a twist from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (ang, vx, vy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream & operator>>(std::istream & is, Twist2D & tf);


    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
        /// Create private elements in the class
        Vector2D x_y;
        double theta;
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief convert a twist to a different frame
        /// \param twist - the twist to change frames
        /// \return a twist in a different frame
        Twist2D operator()(Twist2D twist) const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);


    };

    /// \brief normalize a given angle
    /// \param twist angle to be normalized
    /// \return a normalized angle
    Transform2D integrate_twist(Twist2D twist);

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief normalize a given 2D vector
    /// \param vec 2D vector to be normalized
    /// \return a normalized vector
    Vector2D normalize(Vector2D vec);

    /// \brief normalize a given angle
    /// \param rad angle to be normalized
    /// \return a normalized angle
    double normalize_angle(double rad);
}

#endif
