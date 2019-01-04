#pragma once
#include <vector>

class CollectionU
{
public:
	enum class IterationDir
	{
		NextDir,
		PrevDir
	};

private:
	template<class T, IterationDir itDir>
	struct MoveIndexImp
	{
		static inline int Call(const T& coll, int curr)
		{
			return GetNextIndex(coll, curr);
		}
	};

	template<class T>
	struct MoveIndexImp<T, IterationDir::PrevDir>
	{
		static inline int Call(const T& coll, int curr)
		{
			return GetPrevIndex(coll, curr);
		}
	};

public:
	template<class T>
	static inline int GetNextIndex(const T& coll, int curr)
	{
		return (curr + 1) % coll.size();
	}

	template<class T>
	static inline int GetPrevIndex(const T& coll, int curr)
	{
		auto c = coll.size();
		return (curr - 1 + c) % c;
	}

	template<class T, IterationDir itDir>
	static inline int MoveIndex(const T& coll, int curr)
	{
		return MoveIndexImp<T, itDir>::Call(coll, curr);
	}

	template<class Tcollec, class Tval>
	static inline bool Contains(const Tcollec& collec, const Tval& val)
	{
		return std::find(collec.cbegin(), collec.cend(), val) != collec.cend();
	}

	template<class Tcollec, class Tval>
	static inline void Remove(Tcollec& collec, const Tval& val)
	{
		collec.erase(std::find(collec.begin(), collec.end(), val));
	}
};
