#pragma once

#ifdef FALLDETECTIONDLL_EXPORT
#define FALLDETECTIONDLLAPIS __declspec(dllexport)
#elif  FALLDETECTIONDLL_IMPORT
#define FALLDETECTIONDLLAPIS __declspec(dllimport)
#else
#define FALLDETECTIONDLLAPIS 
#endif