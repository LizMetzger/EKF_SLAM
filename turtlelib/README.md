# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- diff_drive - Class that has functions and constuructors for a differential drive robot

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
         - You could implement this by writing one function that contains all of the logic for normalizing a 2D vector in it and then call it whenever you need it.
         - You could also write an indivicual function for each of the operations involved in normalizing the vector then call the functions in order as you need them.
         - You could also include the function as a member of a class or struct to handle the specific case and variables.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
         - The pros of the first approach are that the code for normalizing is contained within one funciton so its easier to test if the function is working as expected, but the function would probably be longer than an expected function and has too many seperate operations happening within it.
         - This approach follows the guidelines well because it breaks up all the opertations into seperate functions making it easier to test and creating more opportunities to reuse already written and working code. 
         - Putting the function directly into the class or struct is useful because it means that the function will be able to access all of the variables in the class, but it means that the function can't be used on objects of a different type. 


   - Which of the methods would you implement and why?
         - I would probably implement the second approach where each functionality is its own function then I call the functions together to normalize my vector becasue it gives me the most code that I could reuse in other places and best adheres to the C++ guidelines. 

2. What is the difference between a class and a struct in C++?
The main difference is that the members of a class are by default private and can only be accessed by the class
while with structs have members that can be accessed from anywhere at anytime.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
A struct should be used if all the members can vary independently so the use of a class tells people reading the code that there is a invarient. A class should also be used if there is a need to make any of the members private. 
(Referenced C.8 and C.2)

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
The main reason is so that there are no unintended conversions when using a constructor with a single-argument. 
(Refenced C.46)

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
   inv() is declared as const because we don't want to have a change in the object's value or state while the *= operator updates the value of the object itself so it is not declared as const. It is default to make member functions const and the reason that *= isn't const is becasue the purpose of the function is to update the value of the object. (Referenced Con.2)


# Program output:

Enter transform T_{a,b}: 
deg: 90 x: 0 y: 1
Enter transform T_{b,c}: 
deg: 90 x: 1 y: 0
T_{a,b}: deg: 90  x: 0  y: 1
T_{b,a}: deg: -90  x: -1  y: -6.12323e-17
T_{b,c}: deg: 90  x: 1  y: 0
T_{c,b}: deg: -90  x: -6.12323e-17  y: 1
T_{a,c}: deg: 180  x: 6.12323e-17  y: 2
T_{c,a}: deg: -180  x: -1.83697e-16  y: 2
Enter vector v_b: 
1 1
v_bhat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 1.11022e-16]
Enter twist V_b: 
1 1 1
V_a: [1 0 1]
V_b: [1 1 1]
V_c: [1 2 -1]
