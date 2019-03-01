#pragma once
#include "Pool.h"

class ShapeEdge;
class Face;
class ShapePoint;
class Shape;

template<class T>
class ShapeElementPool
{
public:
	template<class... Targs>
	static T& Take(Targs&&... args)
	{
		auto& obj = *m_Pool->GetObject();
		obj.OnTakenFromPool(std::forward<Targs>(args)...);
		return obj;
	}

	static void Return(T& obj)
	{
		obj.OnReturnedToPool();
		m_Pool->Return(&obj);
	}

private:
	static Pool<T*>* const m_Pool;
};

template<class T>
Pool<T*>* const ShapeElementPool<T>::m_Pool = new Pool<T*>([]() { return new T(); }, 500);

using EdgePool = ShapeElementPool<ShapeEdge>;
using FacePool = ShapeElementPool<Face>;
using PointPool = ShapeElementPool<ShapePoint>;
using ShapePool = ShapeElementPool<Shape>;
