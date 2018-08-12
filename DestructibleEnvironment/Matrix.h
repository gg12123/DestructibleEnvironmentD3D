#pragma once

template<int Size>
class Matrix
{
public:
	float M[Size][Size];
};

class Matrix3 : public Matrix<3>
{

};

class Matrix4 : public Matrix<4>
{

};