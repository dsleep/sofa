#pragma once

#include <pocketsphinx.h>
#include <sphinxbase/ad.h>
#include <sphinxbase/err.h>
#include <string>

#ifdef SPHINXLIB_EXPORTS  
#define SPHINXLIB_API __declspec(dllexport)   
#else  
#define SPHINXLIB_API __declspec(dllimport)   
#endif

SPHINXLIB_API typedef struct asr_result {
	char const *hyp;
	int32 confidence;
} asr_result;;

SPHINXLIB_API void sleep_msec(int32);

SPHINXLIB_API asr_result recognize_from_mic(const char*, const char*, const char*);