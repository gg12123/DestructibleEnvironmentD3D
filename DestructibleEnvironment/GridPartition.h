#pragma once
#include "Bounds.h"
#include "Vector3.h"

struct GridRegion
{
	uint32 XStart;
	uint32 YStart;
	uint32 ZStart;

	uint32 XEnd;
	uint32 YEnd;
	uint32 ZEnd;
};

template<class T, uint32 XGridSize, uint32 YGridSize, uint32 ZGridSize>
class GridPartition
{
public:
	T & At(uint32 x, uint32 y, uint32 z)
	{
		return m_Squares[x][y][y];
	}

	void SetSqaureDimensions(float xSquareSize, float ySqaureSize, float zSquareSize, const Vector3& origin)
	{
		m_XSqaureSize = xSquareSize;
		m_YSqaureSize = ySqaureSize;
		m_ZSqaureSize = zSquareSize;

		m_Origin = origin;
	}

	void GetRegionCovered(const Bounds& bounds, GridRegion& region)
	{
		region.XStart = static_cast<uint32>((bounds.GetXMin() - m_Origin.x) / m_XSqaureSize);
		region.XEnd = static_cast<uint32>((bounds.GetXMax() - m_Origin.x) / m_XSqaureSize) + 1U;

		region.YStart = static_cast<uint32>((bounds.GetYMin() - m_Origin.y) / m_YSqaureSize);
		region.YEnd = static_cast<uint32>((bounds.GetYMax() - m_Origin.y) / m_YSqaureSize) + 1U;

		region.ZStart = static_cast<uint32>((bounds.GetZMin() - m_Origin.z) / m_ZSqaureSize);
		region.ZEnd = static_cast<uint32>((bounds.GetZMax() - m_Origin.z) / m_ZSqaureSize) + 1U;
	}

private:
	T m_Squares[XGridSize][YGridSize][ZGridSize];

	float m_XSqaureSize;
	float m_YSqaureSize;
	float m_ZSqaureSize;

	Vector3 m_Origin;
};