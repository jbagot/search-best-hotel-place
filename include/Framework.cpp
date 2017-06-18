
#include "Framework.h"

#include <cassert>
#include <cmath> //for sqrt (square root) function
#include <stdlib.h>
#include <fstream>

#ifdef _WIN32
	#include <windows.h>
	#include <time.h>
#else
	#include <sys/time.h>
#endif

//------------------ Vector2 ----------------------------------------
	
float Vector2::distance(const Vector2& v)
{
	return (v - *this).length();
}

float Vector2::quadraticDistance(const Vector2& v)
{
	return (v - *this).quadraticLength();
}

void Vector2::random(float range)
{
	//rand returns a value between 0 and RAND_MAX
	x = (rand() / (float)RAND_MAX) * 2 * range - range; //value between -range and range
	y = (rand() / (float)RAND_MAX) * 2 * range - range; //value between -range and range
}

Vector2 operator + (const Vector2& a, float v) { return Vector2(a.x + v, a.y + v); }
Vector2 operator - (const Vector2& a, float v) { return Vector2(a.x - v, a.y - v); }
Vector2 operator * (const Vector2& a, float v) { return Vector2(a.x * v, a.y * v); }
Vector2 operator / (const Vector2& a, float v) { return Vector2(a.x / v, a.y / v); }

Vector2 operator + (const Vector2& a, const Vector2& b) { return Vector2(a.x + b.x, a.y + b.y); }
Vector2 operator - (const Vector2& a, const Vector2& b) { return Vector2(a.x - b.x, a.y - b.y); }
Vector2 operator * (const Vector2& a, const Vector2& b) { return Vector2(a.x * b.x, a.y * b.y); }
Vector2 operator / (const Vector2& a, const Vector2& b) { return Vector2(a.x / b.x, a.y / b.y); }

//------------------ Vector3 ----------------------------------------
Vector3::Vector3(const Vector3 &v)
{
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
}

float Vector3::length() 
{
	return sqrt(x*x + y*y + z*z);
}

float Vector3::length() const
{
	return sqrt(x*x + y*y + z*z);
}

void Vector3::normalize()
{
	float len = length();
	x /= len;
	y /= len;
	z /= len;
}

float Vector3::distance(const Vector3& v)
{
	return (v - *this).length();
}

Vector3 Vector3::cross( const Vector3& b )
{
	return Vector3(y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x);
}

float Vector3::dot( const Vector3& v )
{
	return x*v.x + y*v.y + z*v.z;
}

void Vector3::random(float range)
{
	//rand returns a value between 0 and RAND_MAX
	x = (rand() / (float)RAND_MAX) * 2 * range - range; //value between -range and range
	y = (rand() / (float)RAND_MAX) * 2 * range - range; //value between -range and range
	z = (rand() / (float)RAND_MAX) * 2 * range - range; //value between -range and range
}

Vector3 operator + (const Vector3& a, const Vector3& b) { return Vector3(a.x + b.x, a.y + b.y, a.z + b.z ); }
Vector3 operator - (const Vector3& a, const Vector3& b) { return Vector3(a.x - b.x, a.y - b.y, a.z - b.z ); }
Vector3 operator * (const Vector3& a, const Vector3& b) { return Vector3(a.x * b.x, a.y * b.y, a.z * b.z ); }
Vector3 operator / (const Vector3& a, const Vector3& b) { return Vector3(a.x / b.x, a.y / b.y, a.z / b.z ); }

Vector3 operator + (const Vector3& a, float v) { return Vector3(a.x + v, a.y + v, a.z + v); }
Vector3 operator - (const Vector3& a, float v) { return Vector3(a.x - v, a.y - v, a.z - v); }
Vector3 operator * (const Vector3& a, float v) { return Vector3(a.x * v, a.y * v, a.z * v); }
Vector3 operator / (const Vector3& a, float v) { return Vector3(a.x / v, a.y / v, a.z / v); }

Vector3 operator * (const Matrix44& matrix, const Vector3& v)
{   
   float x = matrix.m[0] * v.x + matrix.m[4] * v.y + matrix.m[8] * v.z + matrix.m[12]; 
   float y = matrix.m[1] * v.x + matrix.m[5] * v.y + matrix.m[9] * v.z + matrix.m[13]; 
   float z = matrix.m[2] * v.x + matrix.m[6] * v.y + matrix.m[10] * v.z + matrix.m[14];
   return Vector3(x,y,z);
}


//------------------ Class Matrix44 ----------------------------------------
Matrix44::Matrix44()
{
	clear();
}

void Matrix44::clear()
{
	for(int i=0;i<16;i++)
		m[i]=0.;
}

void Matrix44::setIdentity()
{
	m[0]=1; m[4]=0; m[8]=0; m[12]=0;
	m[1]=0; m[5]=1; m[9]=0; m[13]=0;
	m[2]=0; m[6]=0; m[10]=1; m[14]=0;
	m[3]=0; m[7]=0; m[11]=0; m[15]=1;
}

void Matrix44::transpose()
{
   std::swap(m[1],m[4]); std::swap(m[2],m[8]); std::swap(m[3],m[12]);
   std::swap(m[6],m[9]); std::swap(m[7],m[13]); std::swap(m[11],m[14]);
}

void Matrix44::traslate(float x, float y, float z)
{
	Matrix44 T;
	T.setTranslation(x, y, z);
	*this = *this * T;
}

void Matrix44::rotate( float angle_in_rad, const Vector3& axis )
{
	Matrix44 R;
	R.setRotation(angle_in_rad, axis);
	*this = *this * R;
}

Vector3 Matrix44::rotateVector(const Vector3& v)
{
	Matrix44 temp = *this;
	temp.m[12] = 0.0;
	temp.m[13] = 0.0;
	temp.m[14] = 0.0;
	return temp * v;
}

void Matrix44::traslateLocal(float x, float y, float z)
{
	Matrix44 T;
	T.setTranslation(x, y, z);
	*this = T * *this;
}

void Matrix44::rotateLocal( float angle_in_rad, const Vector3& axis )
{
	Matrix44 R;
	R.setRotation(angle_in_rad, axis);
	*this = R * *this;
}

//To create a traslation matrix
void Matrix44::setTranslation(float x, float y, float z)
{
	setIdentity();
	m[12] = x;
	m[13] = y;
	m[14] = z;
}

//To create a rotation matrix
void Matrix44::setRotation( float angle_in_rad, const Vector3& axis  )
{
	clear();
	Vector3 axis_n = axis;
	axis_n.normalize();

	float c = cos( angle_in_rad );
	float s = sin( angle_in_rad );
	float t = 1 - c;

	M[0][0] = t * axis.x * axis.x + c;
	M[0][1] = t * axis.x * axis.y - s * axis.z;
	M[0][2] = t * axis.x * axis.z + s * axis.y;

	M[1][0] = t * axis.x * axis.y + s * axis.z;
	M[1][1] = t * axis.y * axis.y + c;
	M[1][2] = t * axis.y * axis.z - s * axis.x;

	M[2][0] = t * axis.x * axis.z - s * axis.y;
	M[2][1] = t * axis.y * axis.z + s * axis.x;
	M[2][2] = t * axis.z * axis.z + c;

	M[3][3] = 1.0f;
}

//Multiply a matrix by another and returns the result
Matrix44 Matrix44::operator*(const Matrix44& matrix) const
{
	Matrix44 ret;

	unsigned int i,j,k;
	for (i=0;i<4;i++) 	
	{
		for (j=0;j<4;j++) 
		{
			ret.M[i][j]=0.0;
			for (k=0;k<4;k++) 
				ret.M[i][j] += M[i][k] * matrix.M[k][j];
		}
	}

	return ret;
}

bool Matrix44::inverse()
{
   unsigned int i, j, k, swap;
   float t;
   Matrix44 temp, final;
   final.setIdentity();

   temp = (*this);

   unsigned int m,n;
   m = n = 4;
	
   for (i = 0; i < m; i++)
   {
      // Look for largest element in column

      swap = i;
      for (j = i + 1; j < m; j++)// m or n
	  {
		 if ( fabs(temp.M[j][i]) > fabs( temp.M[swap][i]) )
            swap = j;
	  }
   
      if (swap != i)
      {
         // Swap rows.
         for (k = 0; k < n; k++)
         {
			 std::swap( temp.M[i][k],temp.M[swap][k]);
			 std::swap( final.M[i][k], final.M[swap][k]);
         }
      }

      // No non-zero pivot.  The CMatrix is singular, which shouldn't
      // happen.  This means the user gave us a bad CMatrix.


#define MATRIX_SINGULAR_THRESHOLD 0.0001

      if ( fabsf(temp.M[i][i]) <= MATRIX_SINGULAR_THRESHOLD)
	  {
		  final.setIdentity();
         return false;
	  }
#undef MATRIX_SINGULAR_THRESHOLD

      t = 1.0f/temp.M[i][i];

      for (k = 0; k < n; k++)//m or n
      {
         temp.M[i][k] *= t;
         final.M[i][k] *= t;
      }

      for (j = 0; j < m; j++) // m or n
      {
         if (j != i)
         {
            t = temp.M[j][i];
            for (k = 0; k < n; k++)//m or n
            {
               temp.M[j][k] -= (temp.M[i][k] * t);
               final.M[j][k] -= (final.M[i][k] * t);
            }
         }
      }
   }

   *this = final;

   return true;
}

Vector3 Matrix44::getPosition()
{
	return (*this)*Vector3(0.0f,0.0f,0.0f);
}

//--------------------  Camera ---------------------------//

Camera::Camera()
{
	view_matrix.setIdentity();
	setOrthographic(-100,100,100,-100,-100,100);
}

void Camera::set()
{
	glMatrixMode( GL_MODELVIEW );
	glLoadMatrixf( view_matrix.m );

	glMatrixMode( GL_PROJECTION );
	glLoadMatrixf( projection_matrix.m );
}

Vector3 Camera::getLocalVector(const Vector3& v)
{
	Matrix44 iV = view_matrix;
	iV.inverse();
	Vector3 result = iV.rotateVector(v);
	return result;
}

void Camera::move(Vector3 delta)
{
	Vector3 localDelta = getLocalVector(delta);
	eye = eye - localDelta;
	center = center - localDelta;
	updateViewMatrix();
}

void Camera::rotate(float angle, const Vector3& axis)
{
	Matrix44 R;
	R.setRotation(angle,axis);
	Vector3 new_front = R * (center - eye);
	center = eye + new_front;
	updateViewMatrix();
}

void Camera::setOrthographic(float left, float right, float top, float bottom, float near_plane, float far_plane)
{
	type = ORTHOGRAPHIC;

	this->left = left;
	this->right = right;
	this->top = top;
	this->bottom = bottom;
	this->near_plane = near_plane;
	this->far_plane = far_plane;

	updateProjectionMatrix();
}

void Camera::setPerspective(float fov, float aspect, float near_plane, float far_plane)
{
	type = PERSPECTIVE;

	this->fov = fov;
	this->aspect = aspect;
	this->near_plane = near_plane;
	this->far_plane = far_plane;

	updateProjectionMatrix();
}

void Camera::lookAt(const Vector3& eye, const Vector3& center, const Vector3& up)
{
	this->eye = eye;
	this->center = center;
	this->up = up;

	updateViewMatrix();
}

Vector2 Camera::getSpaceVector(float x, float y)const {
	float width=right-left, height = top-bottom;
	return Vector2(center.x+left+x*width,center.y+bottom + height*y);
}

void Camera::updateViewMatrix()
{
	//We activate the matrix we want to work: modelview
	glMatrixMode(GL_MODELVIEW);

	//We set it as identity
	glLoadIdentity();

	//We find the look at matrix
	gluLookAt( eye.x, eye.y, eye.z, center.x, center.y, center.z, up.x, up.y, up.z);

	//We get the matrix and store it in our app
	glGetFloatv(GL_MODELVIEW_MATRIX, view_matrix.m);
}

void Camera::resize(int width, int heigth)
{
	//We activate the matrix we want to work: projection
	glMatrixMode(GL_PROJECTION);

	//We set it as identity
	glLoadIdentity();

	glViewport(0, 0, width, heigth);

	if (type == PERSPECTIVE)
		gluPerspective(fov, aspect, near_plane, far_plane);
	else
		glOrtho(left,right,bottom,top,near_plane,far_plane);

	//upload to hardware
	glGetFloatv(GL_PROJECTION_MATRIX, projection_matrix.m );
}

//Create a projection matrix
void Camera::updateProjectionMatrix()
{
	//We activate the matrix we want to work: projection
	glMatrixMode(GL_PROJECTION);

	//We set it as identity
	glLoadIdentity();

	if (type == PERSPECTIVE)
		gluPerspective(fov, aspect, near_plane, far_plane);
	else
		glOrtho(left,right,bottom,top,near_plane,far_plane);

	//upload to hardware
	glGetFloatv(GL_PROJECTION_MATRIX, projection_matrix.m );
}

//------------------- Utilities ----------------------

long getTime()
{
	#ifdef WIN32
		return GetTickCount();
	#else
		struct timeval tv;
		gettimeofday(&tv,NULL);
		return (int)(tv.tv_sec*1000 + (tv.tv_usec / 1000));
	#endif
}

void drawCircle(Vector3 center, float radius)
{
	glBegin(GL_LINE_LOOP);

	for (int i=0;i<360;i++)
		glVertex3f(cos(i*DEG2RAD)*radius+center.x,sin(i*DEG2RAD)*radius+center.y,center.z);
	
	glEnd();
}

void drawCircle(Vector2 center, float radius)
{
	glBegin(GL_LINE_LOOP);
	for (int i=0;i<360;i++)
		glVertex3f(cos(i*DEG2RAD)*radius+center.x,sin(i*DEG2RAD)*radius+center.y,0);
	glEnd();
}



float toRad(float degrees) { 
	return (degrees*PI/180.); 
}
