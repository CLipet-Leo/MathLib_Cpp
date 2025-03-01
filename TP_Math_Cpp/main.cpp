#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

#include "Test.h"

int main()
{
	//testProdMat();
	//testInversedMatrix();
	//testTranslation();
	//testRotation();
	testInertia();

	_CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_DEBUG); 
	_CrtDumpMemoryLeaks();
	return 0;
}
