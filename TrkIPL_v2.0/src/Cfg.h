#pragma once

#include <iostream>
//#include <windows.h>	// in Windows
#include <sys/timeb.h>	// in Windows
//#include <ctime>	// in Linux
#include <opencv2/core/core.hpp>

class CCfg
{
public:
	//! full constructor
	CCfg();
	//! default destructor
	~CCfg();

	//! loads configuration file from directory
	void loadCfgFile(void);

	inline cv::Size getFrmSz(void) { return m_oFrmSz; }
	void setFrmSz(cv::Size oFrmSz) { m_oFrmSz = oFrmSz; };
	inline float getFrmRt(void) { return m_fFrmRt; }
	inline void setFrmRt(float fFrmRt) { m_fFrmRt = fFrmRt; }
	inline int getRoiArea(void) { return m_nRoiArea; }
	inline void setRoiArea(int nRoiArea) { m_nRoiArea = nRoiArea; }
	inline int getStFrmCnt(void) { return m_nStFrmCnt; }
	inline void setStFrmCnt(int nFrmCnt) { m_nStFrmCnt = nFrmCnt; }
	//inline SYSTEMTIME getStSysTm() { return m_tStSysTm; }	// in Windows
	//inline void setStSysTm(SYSTEMTIME tSysTm) { m_tStSysTm = tSysTm; }	// in Windows
	inline struct _timeb getStTstp() { return m_stbStTstp; }	// in Windows
	inline void setStTstp(struct _timeb stbStTstp) { m_stbStTstp = stbStTstp; }	// in Windows
	//inline struct timespec getStTstp() { return m_stsStTstp; }	// in Linux
	//inline void setStTstp(struct timespec stsStTstp) { m_stsStTstp = stsStTstp; }	// in Linux

	inline char* getInVdoPth(void) { return m_acInVdoPth; }
	inline char* getInFrmPth(void) { return m_acInFrmPth; }
	inline char* getInRoiPth(void) { return m_acInRoiPth; }
	inline char* getInBgPth(void) { return m_acInBgPth; }
	inline char* getInCamParamPth(void) { return m_acInCamParamPth; }
	inline char* getInCamInParamPth(void) { return m_acInCamInParamPth; }
	inline char* getInDetTxtPth(void) { return m_acInDetTxtPth; }
	inline char* getInSegVdoPth(void) { return m_acInSegVdoPth; }
	inline char* getInSegImgFlrPth(void) { return m_acInSegImgFlrPth; }
	inline char* getOutTrkTxtPth(void) { return m_acOutTrkTxtPth; }
	inline char* getOutFrmFlrPth(void) { return m_acOutFrmFlrPth; }
	inline char* getOutSegFlrPth(void) { return m_acOutSegFlrPth; }
	inline char* getOutDetTxtPth(void) { return m_acOutDetTxtPth; }
	inline char* getOutVdoPth(void) { return m_acOutVdoPth; }
	inline char* getOutImgFlrPth(void) { return m_acOutImgFlrPth; }
	inline char* getOutTstpPth(void) { return m_acOutTstpPth; }
	inline int getInVdoTyp(void) { return m_nInVdoTyp; }
	inline int getInDetTyp(void) { return m_nInDetTyp; }
	inline int getInSegTyp(void) { return m_nInSegTyp; }
	inline int getInCalTyp(void) { return m_nInCalTyp; }
	inline bool getInBgFlg(void) { return m_bInBgFlg; }
	inline bool getInCamInParamFlg(void) { return m_bInCamInParamFlg; }
	inline bool getOutFrmFlg(void) { return m_bOutFrmFlg; }
	inline bool getOutSegFlg(void) { return m_bOutSegFlg; }
	inline bool getOutDetFlg(void) { return m_bOutDetFlg; }
	inline bool getOutVdoFlg(void) { return m_bOutVdoFlg; }
	inline bool getOutImgFlg(void) { return m_bOutImgFlg; }
	inline bool getSelRoiFlg(void) { return m_bSelRoiFlg; }
	inline bool getPltTrkResFlg(void) { return m_bPltTrkResFlg; }
	inline bool getPltDetFlg(void) { return m_bPltDetFlg; }
	inline bool getPltSegFlg(void) { return m_bPltSegFlg; }
	inline double getOutFrmRt(void) { return m_fOutFrmRt; }
	inline float getPltTrajTmSec(void) { return m_fPltTrajTmSec; }
	inline int getPltTrajFrmNum(void) { return m_fPltTrajTmSec * m_fFrmRt; }
	inline int getRszFrmHei(void) { return m_nRszFrmHei; }
	inline int getLenUnit(void) { return m_nLenUnit; }
	inline float getDetRszRat(void) { return m_fDetRszRat; }
	inline float getDetScrThld(void) { return m_fDetScrThld; }
	inline std::vector<std::string> getDetObjCls(void) { return m_vstrDetObjCls; }
	inline bool getDetMltSclTstFlg(void) { return m_bDetMltSclTstFlg; }
	inline float getDetHogHitThld(void) { return m_fDetHogHitThld; }
	inline int getDetHogWinStrdSz(void) { return m_nDetHogWinStrdSz; }
	inline int getDetHogPadSz(void) { return m_nDetHogPadSz; }
	inline float getDetHogScl0(void) { return m_fDetHogScl0; }
	inline int getDetHogGrpThld(void) { return m_nDetHogGrpThld; }
	//inline char* getDetYoloDknetPth(void) { return m_acDetYoloDknetPth; }	// in Linux
	//inline char* getDetYoloDataPth(void) { return m_acDetYoloDataPth; }	// in Linux
	//inline char* getDetYoloCfgPth(void) { return m_acDetYoloCfgPth; }	// in Linux
	//inline char* getDetYoloWgtPth(void) { return m_acDetYoloWgtPth; }	// in Linux
	//inline char* getDetC4Mdl1Pth(void) { return m_acDetC4Mdl1Pth; }
	//inline char* getDetC4Mdl2Pth(void) { return m_acDetC4Mdl2Pth; }
	//inline int getDetC4WidThld(void) { return m_nDetC4WidThld; }
	//inline int getDetC4HeiThld(void) { return m_nDetC4HeiThld; }
	//inline int getDetC4XDiv(void) { return m_nDetC4XDiv; }
	//inline int getDetC4YDiv(void) { return m_nDetC4YDiv; }
	inline float getSegRszRat(void) { return m_fSegRszRat; }
	inline float getSegLrnRt(void) { return m_fSegLrnRt; }
	inline int getSegHstLen(void) { return m_nSegHstLen; }
	inline float getSegDist2Thld(void) { return m_fSegDist2Thld; }
	inline float getSegLbspRelSimiThld(void) { return m_fSegLbspRelSimiThld; }
	inline int getSegDescDistThldOfst(void) { return m_nSegDescDistThldOfst; }
	inline int getSegMinClrDistThld(void) { return m_nSegMinClrDistThld; }
	inline int getSegBgSmpNum(void) { return m_nSegBgSmpNum; }
	inline int getSegReqBgSmpNum(void) { return m_nSegReqBgSmpNum; }
	inline int getSegMovAvgSmpNum(void) { return m_nSegMovAvgSmpNum; }
	inline int getSegFillGapX(void) { return m_nSegFillGapX; }
	inline int getSegFillGapY(void) { return m_nSegFillGapY; }
	inline bool getSegShdwRmvFlg() { return m_bSegShdwRmvFlg; }
	inline float getSegShdwYRatMin(void) { return m_fSegShdwYRatMin; }
	inline float getSegShdwYRatMax(void) { return m_fSegShdwYRatMax; }
	inline int getSegShdwCrDiffThld(void) { return m_nSegShdwCrDiffThld; }
	inline int getSegShdwCbDiffThld(void) { return m_nSegShdwCbDiffThld; }
	inline bool getCalSelVanLnFlg() { return m_bCalSelVanLnFlg; }
	inline cv::Point getCalVr() { return m_oCalVr; }
	inline cv::Point getCalVl() { return m_oCalVl; }
	inline float getCalCamHeiMax(void) { return m_fCalCamHeiMax; };
	inline float getCalCamHeiMin(void) { return m_fCalCamHeiMin; };
	inline int getCalGrdSzR(void) { return m_nCalGrdSzR; };
	inline int getCalGrdSzL(void) { return m_nCalGrdSzL; };
	inline bool getCalEdaOptFlg() { return m_bCalEdaOptFlg; }
	inline std::vector<cv::Point>getCalTstDist2dPtPr(void) { return m_voCalTstDist2dPtPr; }
	inline int getTrkDim(void) { return m_nTrkDim; }
	//inline int getTrkVisTyp(void) { return m_nTrkVisTyp; }
	inline int getTrkCmkShfNum(void) { return m_nTrkCmkShfNum; }
	inline float getTrkNtrTmSecThld(void) { return m_fTrkNtrTmSecThld; }
	inline int getTrkNtrFrmNumThld(void) { return m_fTrkNtrTmSecThld * m_fFrmRt; }
	inline float getTrkLftTmSecThld(void) { return m_fTrkLftTmSecThld; }
	inline int getTrkLftFrmNumThld(void) { return m_fTrkLftTmSecThld * m_fFrmRt; }
	inline float getTrkHypTmSecThld(void) { return m_fTrkHypTmSecThld; }
	inline int getTrkHypFrmNumThld(void) { return m_fTrkHypTmSecThld * m_fFrmRt; }
	inline float getTrkUnmtchTmSecThld(void) { return m_fTrkUnmtchTmSecThld; }
	inline int getTrkUnmtchFrmNumThld(void) { return m_fTrkUnmtchTmSecThld * m_fFrmRt; }
	inline float getTrkTrajTmSecThld(void) { return m_fTrkTrajTmSecThld; }
	inline int getTrkTrajFrmNumThld(void) { return m_fTrkTrajTmSecThld * m_fFrmRt; }

private:
	//! reads char array
	std::string readCharArr(std::string strCfg, int nParamPos);
	//! reads integer number
	int readInt(std::string strCfg, int nParamPos);
	//! reads float number
	float readFlt(std::string strCfg, int nParamPos);
	//! reads bool value
	bool readBool(std::string strCfg, int nParamPos);
	//! reads vector of char arrays
	void readVecStr(std::string strCfg, int nParamPos, std::vector<std::string>& vstrCharArr);
	//! reads 2D point
	cv::Point read2dPt(std::string strCfg, int nParamPos);
	//! reads vector of 2D points
	std::vector<cv::Point> readVecPt(std::string strCfg, int nParamPos);

	//! video frame size
	cv::Size m_oFrmSz;
	//! video frame rate
	float m_fFrmRt;
	//! ROI area
	int m_nRoiArea;
	//! starting frame count
	int m_nStFrmCnt;
	////! starting timestamp
	//SYSTEMTIME m_tStSysTm;	// in Windows
	//! starting timestamp
	struct _timeb m_stbStTstp;	// in Windows
	////! starting timestamp
	//struct timespec m_stsStTstp;	// in Linux
	//! path of input video stream
	char m_acInVdoPth[256];
	//! path of input frame image, necessary when m_nInVdoTyp == 3
	char m_acInFrmPth[256];
	//! path of input ROI image
	char m_acInRoiPth[256];
	//! path of input initial background for segmentation, necessary when m_bInBgFlg == 1
	char m_acInBgPth[256];
	//! path of input text file of camera parameters, necessary when m_nTrkDim == 3
	char m_acInCamParamPth[256];
	//! path of input text file of camera intrinsic parameters, necessary when m_bInCamInParamFlg == true
	char m_acInCamInParamPth[256];
	//! path of input text file of object detection, necessary when m_bInDetTyp == 0
	char m_acInDetTxtPth[256];
	//! path of input video file of segmentation, necessary when m_bInSegTyp == 0
	char m_acInSegVdoPth[256];
	//! path of input image folder of segmentation, necessary when m_bInSegTyp == 1
	char m_acInSegImgFlrPth[256];
	//! path of output text file of tracking results
	char m_acOutTrkTxtPth[256];
	//! path of folder for output (original) frame image files, necessary when m_bOutFrmFlg == true
	char m_acOutFrmFlrPth[256];
	//! path of folder for output segmentation masks, necessary when m_bOutSegFlg == true
	char m_acOutSegFlrPth[256];
	//! path of output text file of object detection, necessary when m_bOutDetFlg == true
	char m_acOutDetTxtPth[256];
	//! path of output video file, necessary when m_bOutVdoFlg == true
	char m_acOutVdoPth[256];
	//! path of folder for output image files, necessary when m_bOutImgFlg == true
	char m_acOutImgFlrPth[256];
	//! path of output timestamp text file, necessary when m_nInVdoTyp > 0
	char m_acOutTstpPth[256];
	//! type of input video source: 0: video file; 1: USB camera; 2: IP camera; 3: updating image
	int m_nInVdoTyp;
	//! type of input detection: 0: text file; 1: online HOG (person only); 2: online YOLO v2
	int m_nInDetTyp;
	//! type of input segmentation: 0: video file; 1: image folder; 2: online KNN; 3: online MOG2; 4: online SuBSENSE
	int m_nInSegTyp;
	//! type of input calibration: 0: text file; 1: manual calibration; 2: self-calibration from 2D tracking, necessary when m_nTrkDimm_nTrkDim == 3
	int m_nInCalTyp;
	//! flag of input initial background for segmentation, necessary when m_nInSegTyp > 1
	bool m_bInBgFlg;
	//! flag of input text file of camera intrinsic parameters, necessary when m_nInCalTyp > 0
	bool m_bInCamInParamFlg;
	//! flag of output (original) frame image files
	bool m_bOutFrmFlg;
	//! flag of output segmentation masks
	bool m_bOutSegFlg;
	//! flag of output detection results as txt file
	bool m_bOutDetFlg;
	//! flag of output video file
	bool m_bOutVdoFlg;
	//! flag of output image files
	bool m_bOutImgFlg;
	//! flag of selecting ROI image
	bool m_bSelRoiFlg;
	//! flag of plotting tracking results
	bool m_bPltTrkResFlg;
	//! flag of plotting object detection
	bool m_bPltDetFlg;
	//! flag of plotting segmentation
	bool m_bPltSegFlg;
	//! frame rate of the output video, necessary when m_bOutVdoFlg = true and m_nInVdoTyp > 0
	double m_fOutFrmRt;
	//! time window in seconds for plotting previous trajectory, necessary when m_bPltTrkResFlg == true
	float m_fPltTrajTmSec;
	//! resized video frame height (-1: original size)
	int m_nRszFrmHei;
	//! the length unit, 10 or 1000 (1 cm = 10 mm, 1 m = 1000 mm)
	int m_nLenUnit;
	//! the ratio to resize the original image for human detection, necessary when m_nInDetTyp > 0
	float m_fDetRszRat;
	//! threshold of detection score (in percentage), necessary when m_nInDetTyp == 0 or 2
	float m_fDetScrThld;
	//! list of object classes, necessary when m_nInDetTyp == 0 or 2
	std::vector<std::string> m_vstrDetObjCls;
	 //! flag of multi-scale testing, necessary when m_nInDetTyp > 0
    bool m_bDetMltSclTstFlg;
	//! threshold for the feature distance with SVM classifying plane for HOG human detection, necessary when m_nInDetTyp == 1
	float m_fDetHogHitThld;
	//! size of window stride for HOG human detection, necessary when m_nInDetTyp == 1
	int m_nDetHogWinStrdSz;
	//! size of the mock parameter for HOG human detection, necessary when m_nInDetTyp == 1
	int m_nDetHogPadSz;
	//! coefficient of the detection window increase for HOG human detection, necessary when m_nInDetTyp == 1
	float m_fDetHogScl0;
	//! coefficient to regulate the similarity threshold for HOG human detection, necessary when m_nInDetTyp == 1
	int m_nDetHogGrpThld;
	////! path of darknet for YOLO v2 detection, necessary when m_nInDetTyp == 2
    //char m_acDetYoloDknetPth[256];	// in Linux
	////! path of data file for YOLO v2 detection, necessary when m_nInDetTyp == 2
    //char m_acDetYoloDataPth[256];	// in Linux
    ////! path of configuration file for YOLO v2 detection, necessary when m_nInDetTyp == 2
    //char m_acDetYoloCfgPth[256];	// in Linux
    ////! path of weights file for YOLO v2 detection, necessary when m_nInDetTyp == 2
    //char m_acDetYoloWgtPth[256];	// in Linux
	////! path of the 1st model for C4 human detection, necessary when m_nInDetTyp == 2
	//char m_acDetC4Mdl1Pth[256];
	////! path of the 2nd model for C4 human detection, necessary when m_nInDetTyp == 2
	//char m_acDetC4Mdl2Pth[256];
	////! width threshold of detected object for C4 human detection, necessary when m_nInDetTyp == 2
	//int m_nDetC4WidThld;
	////! height threshold of detected object for C4 human detection, necessary when m_nInDetTyp == 2
	//int m_nDetC4HeiThld;
	////! division along x direction for C4 human detection, necessary when m_nInDetTyp == 2
	//int m_nDetC4XDiv;
	////! division along y direction for C4 human detection, necessary when m_nInDetTyp == 2
	//int m_nDetC4YDiv;
	//! the ratio to resize the original image for segmentation
	float m_fSegRszRat;
	//! the overridden learning rate (-1.0: default/adaptive), necessary when m_nInSegTyp > 1
	float m_fSegLrnRt;
	//! the history length for KNN and MOG2, necessary when m_nInSegTyp == 2 or 3
	int m_nSegHstLen;
	//! threshold on the squared distance for KNN (default: 400.0) and MOG2 (default: 16.0), necessary when m_nInSegTyp == 2 or 3
	float m_fSegDist2Thld;
	//! LBSP relative internal threshold for SuBSENSE, necessary when m_nInSegTyp == 4
	float m_fSegLbspRelSimiThld;
	//! absolute descriptor distance threshold offset for SuBSENSE, necessary when m_nInSegTyp == 4
	int m_nSegDescDistThldOfst;
	//! absolute minimal color distance threshold for SuBSENSE, necessary when m_nInSegTyp == 4
	int m_nSegMinClrDistThld;
	//! number of different samples per pixel/block to be taken from input frames to build the background model for SuBSENSE, necessary when m_nInSegTyp == 4
	int m_nSegBgSmpNum;
	//! number of similar samples needed to consider the current pixel/block as 'background' for SuBSENSE, necessary when m_nInSegTyp == 4
	int m_nSegReqBgSmpNum;
	//! number of samples to use to compute the learning rate of moving averages for SuBSENSE, necessary when m_nInSegTyp == 4
	int m_nSegMovAvgSmpNum;
	//! length of gap between blobs in x direction to be filled, necessary when m_nInSegTyp > 1
	int m_nSegFillGapX;
	//! length of gap between blobs in y direction to be filled, necessary when m_nInSegTyp > 1
	int m_nSegFillGapY;
	//! flag of shadow removal, necessary when m_nInSegTyp > 1
	bool m_bSegShdwRmvFlg;
	//! minimum Y ratio for shadow removal, necessary when m_bSegShdwrRmvFlg == true
	float m_fSegShdwYRatMin;
	//! maximum Y ratio for shadow removal, necessary when m_bSegShdwrRmvFlg == true
	float m_fSegShdwYRatMax;
	//! maximum Cr difference for shadow removal, necessary when m_bSegShdwrRmvFlg == true
	int m_nSegShdwCrDiffThld;
	//! maximum Cb difference for shadow removal, necessary when m_bSegShdwrRmvFlg == true
	int m_nSegShdwCbDiffThld;
	//! flag of selecting vanishing lines on the ground plane, necessary when m_nInCalTyp == 1
	bool m_bCalSelVanLnFlg;
	//! given vanishing point Vr, necessary when m_bCalSelVanLnFlg == false
	cv::Point m_oCalVr;
	//! given vanishing point Vl, necessary when m_bCalSelVanLnFlg == false
	cv::Point m_oCalVl;
	//! the maximum height of camera in m_nLenUnit, necessary when m_nInCalTyp > 1
	float m_fCalCamHeiMax;
	//! the minimum height of camera in m_nLenUnit, necessary when m_nInCalTyp > 1
	float m_fCalCamHeiMin;
	//! size of the 3D grid (in m_nLenUnit) on ground plane along R axis, necessary when m_nInCalTyp > 0
	int m_nCalGrdSzR;
	//! size of the 3D grid (in m_nLenUnit) on ground plane along L axis, necessary when m_nInCalTyp > 0
	int m_nCalGrdSzL;
	//! flag of EDA optimization for camera calibration, necessary when m_nInCalTyp > 0
	bool m_bCalEdaOptFlg;
	//! pair(s) of 2D points for testing 3D distance (size must be an even number)
	std::vector<cv::Point> m_voCalTstDist2dPtPr;
	//! dimension of tracking: 3: 3D; 2: 2D
	int m_nTrkDim;
	////! type of visual tracking: -1: None; 0: Boosting; 1: MIL; 2: KCF; 3: TLD; 4: MedianFlow; 5: GOTURN
	//int m_nTrkVisTyp;
	//! the number of shifts in each direction for CMK tracking
	int m_nTrkCmkShfNum;
	//! threshold of existing time in seconds for entering objects
	float m_fTrkNtrTmSecThld;
	//! threshold of time in seconds to keep temporarily left objects
	float m_fTrkLftTmSecThld;
	//! threshold of time in seconds that the objects are tracked as hypothesis
	float m_fTrkHypTmSecThld;
	//! threshold of time in seconds to keep tracking unmatched objects
	float m_fTrkUnmtchTmSecThld;
	//! threshold of time in seconds to keep past trajectory for each object
	float m_fTrkTrajTmSecThld;
};
