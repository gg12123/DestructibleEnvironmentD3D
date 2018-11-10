#pragma once

template<class T>
class ObjectWithHash
{
public:
	static void ResetNextHashCounter()
	{
		NextHash = 0;
	}

	void ResetHash()
	{
		m_Hash = UnAssignedHash;
	}

	bool HashIsAssigned() const
	{
		return m_Hash != UnAssignedHash;
	}

	int GetHash() const
	{
		return m_Hash;
	}

	void TryAssignHash()
	{
		if (!HashIsAssigned())
		{
			m_Hash = NextHash;
			NextHash++;
		}
	}

	void AssignHash()
	{
		m_Hash = NextHash;
		NextHash++;
	}

private:
	static int NextHash;
	static constexpr int UnAssignedHash = -1;

	int m_Hash = UnAssignedHash;
};
