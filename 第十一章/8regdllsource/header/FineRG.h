#ifndef _FINERGGH__ 
#define _FINERGGH__
#include "stdafx.h"

namespace RG
{
	/**
	* \class FineRG
	*this class was designed for fine registration
	*/
	 class RGlib_EXPORT  FineRG:public QObject
	{
		 Q_OBJECT
	public:
		FineRG();
		~FineRG();

signals:

	};
}


#endif