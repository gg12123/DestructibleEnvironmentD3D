// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "ObjectWithHash.h"
#include <vector>
#include <assert.h>

class Shape;

/**
 * 
 */
class ShapePoint : public ObjectWithHash<ShapePoint>
{
public:

private:
	Vector3 m_Point;
};
