#pragma once
#include "DebugLogger.h"

class Loggers
{
public:
	static DebugLogger & Physics()
	{
		static DebugLogger instance;
		instance.TrySetPath("C:\\Users\\gg_11\\AppData\\Local\\Packages\\fe8ccf18-f7f1-4af7-bf09-85c7a31d658b_r9w0nqymqe1rc\\LocalState\\Physics.txt");
		return instance;
	}
};
