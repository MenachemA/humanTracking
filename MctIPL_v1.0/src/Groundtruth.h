#pragma once

#include <opencv\cv.h>

struct GroundTruth{
	int cam_id;
	int label_id;
	int fr;
	CvRect rect;
};

