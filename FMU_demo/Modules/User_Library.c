#include "User_Library.h"



float constrain_float(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}







