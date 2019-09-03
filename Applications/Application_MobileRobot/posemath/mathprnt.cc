
/*
  Modification History:

  14-Apr-1997  FMP created from C++ portion of mathprnt.c
*/

#if HAVE_CONFIG_H
#include "rcs_config.h"
#if !defined(HAVE_STD) && !defined(HAVE_IOSTREAM_H)
#define NO_IOSTREAM
#endif
#endif

#ifndef NO_IOSTREAM

#if HAVE_CONFIG_H
#ifdef HAVE_NAMESPACES
using namespace std;
#endif // HAVE_NAMESPACES
#ifdef HAVE_STD
#include <iostream>
#include <cstdio>
#else //HAVE_CONFIG_H
#include <iostream.h>
#endif // HAVE_STD
#else
#include <iostream.h>
#endif // HAVE_CONFIG_H

#include "posemath.h"
#include "mathprnt.h"



ostream & operator << (ostream & stream, PM_CARTESIAN v)
{
  stream << v.x << "\t" << v.y << "\t" << v.z ;

  return stream;
}

ostream & operator << (ostream & stream, PM_SPHERICAL s)
{
  stream << s.theta << "\t" << s.phi << "\t" << s.r ;

  return stream;
}

ostream & operator << (ostream & stream, PM_CYLINDRICAL c)
{
  stream << c.theta << "\t" << c.r << "\t" << c.z ;

  return stream;
}

ostream & operator << (ostream & stream, PM_QUATERNION q)
{
  stream << q.s << "\t" << q.x << "\t" << q.y << "\t" << q.z ;

  return stream;
}

ostream & operator << (ostream & stream, PM_ROTATION_VECTOR r)
{
  stream << r.s << "\t" << r.x << "\t" << r.y << "\t" << r.z ;

  return stream;
}

ostream & operator << (ostream & stream, PM_ROTATION_MATRIX m)
{
  int row, col;

  for (col = 0; col < 3; col++)
  {
    for (row = 0; row < 3; row++)
    {
      stream << m[row][col] << "\t";
    }
    stream << endl;
  }

  return stream;
}

ostream & operator << (ostream & stream, PM_EULER_ZYZ zyz)
{
  stream << zyz.z << "\t" << zyz.y << "\t" << zyz.zp ;

  return stream;
}

ostream & operator << (ostream & stream, PM_EULER_ZYX zyx)
{
  stream << zyx.z << "\t" << zyx.y << "\t" << zyx.x ;

  return stream;
}

ostream & operator << (ostream & stream, PM_RPY rpy)
{
  stream << rpy.r << "\t" << rpy.p << "\t" << rpy.y ;

  return stream;
}

ostream & operator << (ostream & stream, PM_POSE pose)
{
  stream << pose.tran << "\t" << pose.rot ;

  return stream;
}

ostream & operator << (ostream & stream, PM_HOMOGENEOUS hom)
{
  int row, col;

  for (col = 0; col < 4; col++)
  {
    for (row = 0; row < 4; row++)
    {
      stream << hom[row][col] << "\t";
    }
    stream << endl;
  }

  return stream;
}

 // NO_IOSTREAM not defined
#endif
