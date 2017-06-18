#ifndef FRAMEWORK //macros to ensure the code is included once
#define FRAMEWORK

#include <vector>
#include <cmath> 
#include <time.h>
#include <math.h>

// Glew, GLUT, GL includes
#include <GL/glew.h>
#ifdef _WIN32
    #include <GL/wglew.h>
#endif

#ifdef unix
    #if defined(__APPLE__) || defined(MACOSX)
       #include <OpenGL/OpenGL.h>
       #include <GLUT/freeglut.h>
    #else
       #include <GL/freeglut.h>
       #include <GL/glx.h>
    #endif
#else
    #include <GL/freeglut.h>
#endif

#define DEG2RAD 0.0174532925f
#define PI		3.1415926535897932384f
//------------------ Vector2 ----------------------------------------
class Vector2
{
public:
	union
	{
		struct { float x,y; }; 
		float v[2];
	};

	Vector2() { x = y = 0.0f; }
	Vector2(float _x, float _y) : x(_x), y(_y) { }

	float length() { return sqrt(x*x + y*y); }
	float length() const { return sqrt(x*x + y*y); }
	float quadraticLength() const { return x*x + y*y; }

	void set(float x, float y) { this->x = x; this->y = y; }

	void normalize() { *this /= length(); }

	float distance(const Vector2& v);
	float quadraticDistance(const Vector2& v);
	void random(float range);

	Vector2& operator += (float v) { x+=v; y+=v; return *this; }
	Vector2& operator -= (float v) { x-=v; y-=v; return *this; }
	Vector2& operator *= (float v) { x*=v; y*=v; return *this; }
	Vector2& operator /= (float v) { x/=v; y/=v; return *this; }
};

Vector2 operator + (const Vector2& a, float v);
Vector2 operator - (const Vector2& a, float v);
Vector2 operator * (const Vector2& a, float v);
Vector2 operator / (const Vector2& a, float v);

Vector2 operator + (const Vector2& a, const Vector2& b);
Vector2 operator - (const Vector2& a, const Vector2& b);
Vector2 operator * (const Vector2& a, const Vector2& b);
Vector2 operator / (const Vector2& a, const Vector2& b);

//------------------ Vector3 ----------------------------------------

class Vector3
{
public:
	union
	{
		struct { float x,y,z; };
		float v[3];
	};

	Vector3() { x = y = z = 0.0f; }
	Vector3(const Vector3 &v);
	Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { }

	float length();
	float length() const;

	void set(float x, float y, float z) { this->x = x; this->y = y; this->z = z; }
	void set(const Vector3 &c) { this->x = c.x; this->y = c.y; this->z = c.z; }

	void normalize();
	void random(float range);
	float distance(const Vector3& v);

	Vector3 cross( const Vector3& v );
	float dot( const Vector3& v );

	bool equals(const Vector3& v) { return (this->x==v.x && this->y==v.y); }

	Vector3& operator += (float v) { x+=v; y+=v; z+=v; return *this; }
	Vector3& operator -= (float v) { x-=v; y-=v; z-=v; return *this; }
	Vector3& operator *= (float v) { x*=v; y*=v; z*=v; return *this; }
	Vector3& operator /= (float v) { x/=v; y/=v; z/=v; return *this; }
};

Vector3 operator + (const Vector3& a, const Vector3& b);
Vector3 operator - (const Vector3& a, const Vector3& b);
Vector3 operator * (const Vector3& a, const Vector3& b);
Vector3 operator / (const Vector3& a, const Vector3& b);

Vector3 operator + (const Vector3& a, float v);
Vector3 operator - (const Vector3& a, float v);
Vector3 operator * (const Vector3& a, float v);
Vector3 operator / (const Vector3& a, float v);

//------------------ Matrix44 ----------------------------------------
class Matrix44
{
	public:

		//This matrix works in 
		union { //allows to access the same var using different ways
			float M[4][4]; //[row][column]
			float m[16];
		};

		Matrix44();
		Matrix44(const float* v);

		void clear();
		void setIdentity();
		void transpose();

		bool inverse();

		//rotate only
		Vector3 rotateVector( const Vector3& v);

		//traslate using world coordinates
		void traslate(float x, float y, float z);
		void rotate( float angle_in_rad, const Vector3& axis  );
		void traslateLocal(float x, float y, float z);
		void rotateLocal( float angle_in_rad, const Vector3& axis  );

		//create a transformation matrix from scratch
		void setTranslation(float x, float y, float z);
		void setRotation( float angle_in_rad, const Vector3& axis );

		Matrix44 operator * (const Matrix44& matrix) const;
		Vector3 getPosition();
};

Vector3 operator * (const Matrix44& matrix, const Vector3& v);

//--------------------------- Camera ------------------------------------

class Camera
{
public:
	enum { PERSPECTIVE, ORTHOGRAPHIC };

	char type; //camera type

	//vectors to define the orientation of the camera
	Vector3 eye;
	Vector3 center;
	Vector3 up;

	//properties of the projection of the camera
	float fov;			//view angle
	float aspect;		//aspect ratio
	float near_plane;	//near plane
	float far_plane;	//far plane

	//for orthogonal projection
	float left,right,top,bottom;

	//matrices
	Matrix44 view_matrix;
	Matrix44 projection_matrix;

	Camera();
	void set();

	//move in camera space
	void move(Vector3 delta);

	void rotate(float angle, const Vector3& axis);

	Vector3 getLocalVector(const Vector3& v);
	Vector2 getSpaceVector(float x, float y)const;

	void setPerspective(float fov, float aspect, float near_plane, float far_plane);
	void resize(int width, int heigth);
	void setOrthographic(float left, float right, float top, float bottom, float near_plane, float far_plane);
	void lookAt(const Vector3& eye, const Vector3& center, const Vector3& up);

	void updateViewMatrix();
	void updateProjectionMatrix();
};

//--------------BMP---------------
#define IMG_OK              0x1
#define IMG_ERR_NO_FILE     0x2
#define IMG_ERR_MEM_FAIL    0x4
#define IMG_ERR_BAD_FORMAT  0x8
#define IMG_ERR_UNSUPPORTED 0x40

//--------------utils---------------
long getTime();
void drawCircle(Vector3 center, float radius);
void drawCircle(Vector2 center, float radius);
float toRad(float degrees);

#endif

