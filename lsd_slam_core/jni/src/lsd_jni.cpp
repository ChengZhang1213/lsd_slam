#include <string.h>
#include <jni.h>
#include <android/log.h>

#include "LiveSLAMWrapper.h"
#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "util/Intrinsics.h"
#include "SlamSystem.h"
#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include "util/Undistorter.h"
#include "util/RawLogReader.h"
#include "opencv2/opencv.hpp"
#include "util/logger.h"
#include "sophus/sim3.hpp"
#include "Android/AndroidOutput3DWrapper.h"
#include "misc.h"


// TODO: remove hard code
#define CALIB_FILE "/sdcard/LSD/cameraCalibration.cfg"
#define IMAGE_DIR "/sdcard/LSD/images"

using namespace lsd_slam;
ThreadMutexObject<bool> lsdDone(false);
std::vector<std::string> files;
int w, h, w_inp, h_inp;
RawLogReader * logReader = 0;
int numFrames = 0;
Undistorter* undistorter = NULL;
SlamSystem * slamSystem = NULL;
Output3DWrapper* outputWrapper = NULL;
Sophus::Matrix3f K;
bool dumpCount = 0;

void run_once(SlamSystem * system, Undistorter* undistorter, Output3DWrapper* outputWrapper, Sophus::Matrix3f K)
{    
    
}

void run(SlamSystem * system, Undistorter* undistorter, Output3DWrapper* outputWrapper, Sophus::Matrix3f K) {
    LOGD("----------run-----------\n");
    // get HZ
    double hz = 30;

    cv::Mat image = cv::Mat(h, w, CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; i < numFrames; i++)
    {
        if(lsdDone.getValue())
            break;

        cv::Mat imageDist = cv::Mat(h, w, CV_8U);

        if(logReader)
        {
            logReader->getNext();

            cv::Mat3b img(h, w, (cv::Vec3b *)logReader->rgb);

            cv::cvtColor(img, imageDist, CV_RGB2GRAY);
        }
        else
        {
            imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);

            if(imageDist.rows != h_inp || imageDist.cols != w_inp)
            {
                if(imageDist.rows * imageDist.cols == 0)
                    printf("failed to load image %s! skipping.\n", files[i].c_str());
                else
                    printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                            files[i].c_str(),
                            w,h,imageDist.cols, imageDist.rows);
                continue;
            }
        }

        assert(imageDist.type() == CV_8U);

        undistorter->undistort(imageDist, image);

        assert(image.type() == CV_8U);
        if(runningIDX == 0)
        {
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
        }
        else
        {
            system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp);
        }
        
        //printTrans(system->getCurrentPoseEstimateScale().matrix());
        //gui.pose.assignValue(system->getCurrentPoseEstimateScale());

        runningIDX++;
        fakeTimeStamp+=0.03;
 
        if(fullResetRequested)
        {
            LOGD("FULL RESET!\n");
            delete system;

            system = new SlamSystem(w, h, K, doSlam);
            system->setVisualization(outputWrapper);

            fullResetRequested = false;
            runningIDX = 0;
        }
    }
    
    
}


extern "C"{
JavaVM* jvm = NULL;

//init LSD
JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeInit(JNIEnv* env, jobject thiz) {
	LOGD("nativeInit");
    //init jni
	env->GetJavaVM(&jvm);
	
	std::string calibFile = CALIB_FILE;
	undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
	if(undistorter == 0) {
		LOGE("need camera calibration file! (set using -c FILE)\n");
		exit(0);
	}

	w = undistorter->getOutputWidth();
	h = undistorter->getOutputHeight();

	w_inp = undistorter->getInputWidth();
	h_inp = undistorter->getInputHeight();
	LOGD("w=%d, h=%d, w_inp=%d, h_inp=%d\n", w, h, w_inp, h_inp);

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
	LOGD("fx=%f, fy=%f, cx=%f, cy=%f\n", fx, fy, cx, cy);

	Resolution::getInstance(w, h);
	Intrinsics::getInstance(fx, fy, cx, cy);

	outputWrapper = new AndroidOutput3DWrapper(w, h);

	// make slam system
	slamSystem = new SlamSystem(w, h, K, doSlam);
	slamSystem->setVisualization(outputWrapper);

    // open image files: first try to open as file.
	std::string source = IMAGE_DIR;

	Bytef * decompressionBuffer = new Bytef[Resolution::getInstance().numPixels() * 2];
    IplImage * deCompImage = 0;

    if(source.substr(source.find_last_of(".") + 1) == "klg") {
        logReader = new RawLogReader(decompressionBuffer,
                                     deCompImage,
                                     source);
        numFrames = logReader->getNumFrames();
    }
    else {
        if(getdir(source, files) >= 0) {
            LOGD("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
        }
        else if(getFile(source, files) >= 0) {
            LOGD("found %d image files in file %s!\n", (int)files.size(), source.c_str());
        }
        else {
            LOGD("could not load file list! wrong path / file?\n");
        }
        numFrames = (int)files.size();
    }
}

// clean up
JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeDestroy(JNIEnv* env, jobject thiz) {
	LOGD("nativeDestroy");
	lsdDone.assignValue(true);
}

// init OpenGL
JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeInitGL(JNIEnv* env, jobject thiz) {
	LOGD("nativeInitGL");
	boost::thread lsdThread(run, slamSystem, undistorter, outputWrapper, K);
}

//resize window (might only work once)
JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeResize(JNIEnv* env, jobject thiz , jint w, jint h) {
	LOGD("nativeResize: w=%d, h=%d\n", w, h);
}

//render and process a new frame
JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeRender(JNIEnv* env, jobject thiz) {
    LOGD("nativeRender");
    
    run_once(slamSystem, undistorter, NULL, K);
}

//forward keyboard to LSD
JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeKey(JNIEnv* env, jobject thiz, jint keycode) {
    LOGD("nativeKey: keycode=%d\n", keycode);
}

JNIEXPORT jfloatArray JNICALL
Java_com_tc_tar_TARNativeInterface_nativeGetIntrinsics(JNIEnv* env, jobject thiz) {
    LOGD("nativeKey: nativeGetIntrinsics\n");
    
    jfloatArray result;
    result = env->NewFloatArray(4);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }
    
    jfloat array1[4];
    array1[0] = Intrinsics::getInstance().cx();
    array1[1] = Intrinsics::getInstance().cy();
    array1[2] = Intrinsics::getInstance().fx();
    array1[3] = Intrinsics::getInstance().fy();
    
    env->SetFloatArrayRegion(result, 0, 4, array1);
    return result;
}

JNIEXPORT jintArray JNICALL
Java_com_tc_tar_TARNativeInterface_nativeGetResolution(JNIEnv* env, jobject thiz) {
    LOGD("nativeKey: nativeGetResolution\n");
    jintArray result;
    result = env->NewIntArray(2);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }
    jint array1[2];
    array1[0] = Resolution::getInstance().width();
    array1[1] = Resolution::getInstance().height();

    env->SetIntArrayRegion(result, 0, 2, array1);
    return result;
}

JNIEXPORT jfloatArray JNICALL
Java_com_tc_tar_TARNativeInterface_nativeGetCurrentPose(JNIEnv* env, jobject thiz) {
    //LOGD("nativeKey: nativeGetPose\n");
    assert (slamSystem != NULL);
    jfloatArray result;
    int length = 16;
    result = env->NewFloatArray(length);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    Sophus::Matrix4f m = slamSystem->getCurrentPoseEstimateScale().matrix();
    GLfloat* pose = m.data();
    jfloat array1[length];
    for (int i=0; i<length; ++i) {
        array1[i] = pose[i];    // TODO: use memcpy
    }

    env->SetFloatArrayRegion(result, 0, length, array1);
    return result;
}

JNIEXPORT jfloatArray JNICALL
Java_com_tc_tar_TARNativeInterface_nativeGetAllKeyFramePoses(JNIEnv* env, jobject thiz) {
    assert (slamSystem != NULL);
    AndroidOutput3DWrapper* output = (AndroidOutput3DWrapper*)outputWrapper;
    std::map<int, Keyframe *>& keyframes = output->getKeyframes().getReference();

    jfloatArray result;
    int length = keyframes.size() * 16;
    result = env->NewFloatArray(length);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }
    jfloat array1[length];

    int offset = 0;
    for(std::map<int, Keyframe *>::iterator i = keyframes.begin(); i != keyframes.end(); ++i)
    {
        //Don't render first five, according to original code
        if(i->second->initId >= cutFirstNKf)
        {
            Sophus::Matrix4f m = i->second->camToWorld.matrix();
            GLfloat* pose = m.data();
            for (int i=0; i<16; ++i) {
                array1[i + offset] = pose[i];   // TODO: use memcpy
            }
            offset += 16;
        }
    }
    env->SetFloatArrayRegion(result, 0, length, array1);
    return result;
}

JNIEXPORT jobjectArray JNICALL
Java_com_tc_tar_TARNativeInterface_nativeGetAllKeyFrames(JNIEnv* env, jobject thiz) {
    //LOGD("nativeGetAllKeyFrames\n");
    assert (slamSystem != NULL);
    AndroidOutput3DWrapper* output = (AndroidOutput3DWrapper*)outputWrapper;
    ThreadMutexObject<std::map<int, Keyframe *> >& keyframes = output->getKeyframes();
    
    jclass classKeyFrame = env->FindClass("com/tc/tar/LSDKeyFrame");
    std::list<jobject> objectList;

    boost::mutex::scoped_lock lock(keyframes.getMutex());
#if 0
    if (keyframes.getReference().size() > 20 && dumpCount == 0) {
        dumpCloudPoint(keyframes.getReference());
        dumpCount++;
    }
#endif    
    for(std::map<int, Keyframe *>::iterator i = keyframes.getReference().begin(); i != keyframes.getReference().end(); ++i) {
        //Don't render first five, according to original code
        if(i->second->initId >= cutFirstNKf) {
            Keyframe::MyVertex* vertices = i->second->computeVertices(true);

            int pointNum = i->second->points;
            jfloat* points = new jfloat[pointNum * 3];
            jint* colors = new jint[pointNum];
            int points_offset = 0;
            int colors_offset = 0;
            for (int j=0; j<pointNum; ++j) {
                memcpy(points + points_offset, vertices[j].point, 3 * sizeof(float));
                colors[colors_offset] = (vertices[j].color[3] << 24) + (vertices[j].color[0] << 16) + (vertices[j].color[1] << 8) + vertices[j].color[2];
                points_offset += 3;
                colors_offset++;
            }

            // new KeyFrame object
            jmethodID initMethodID = env->GetMethodID(classKeyFrame, "<init>", "()V");
            if (initMethodID == NULL) {
                LOGE("Cannot find initMethodID!!!\n");
                return NULL;
            }
            jobject keyFrameObject = env->NewObject(classKeyFrame, initMethodID);
            if (keyFrameObject == NULL) {
                LOGE("keyFrameObject is NULL!!!\n");
                return NULL;
            }

            // set pose
            jfloatArray poseArray = env->NewFloatArray(16);
            Sophus::Matrix4f m = i->second->camToWorld.matrix();
            env->SetFloatArrayRegion(poseArray, 0, 16, m.data());
            jfieldID poseFieldID = env->GetFieldID(classKeyFrame, "pose", "[F");
            assert (poseFieldID != NULL);
            env->SetObjectField(keyFrameObject, poseFieldID, poseArray);

            // set pointCount
            jint pointCount = pointNum;
            jfieldID pointCountFieldID = env->GetFieldID(classKeyFrame, "pointCount", "I");
            assert (pointCountFieldID != NULL);
            env->SetIntField(keyFrameObject, pointCountFieldID, pointCount);

            // set points
            jfloatArray pointsArray = env->NewFloatArray(pointNum * 3);
            env->SetFloatArrayRegion(pointsArray, 0, pointNum * 3, points);
            jfieldID pointsFieldID = env->GetFieldID(classKeyFrame, "worldPoints", "[F");
            assert (pointsFieldID != NULL);
            env->SetObjectField(keyFrameObject, pointsFieldID, pointsArray);

            // set colors
            jintArray colorsArray = env->NewIntArray(pointNum);
            env->SetIntArrayRegion(colorsArray, 0, pointNum, colors);
            jfieldID colorsFieldID = env->GetFieldID(classKeyFrame, "colors", "[I");
            assert (colorsFieldID != NULL);
            env->SetObjectField(keyFrameObject, colorsFieldID, colorsArray);

            objectList.push_back(keyFrameObject);

            delete points;
            points = NULL;
            delete colors;
            colors = NULL;
        }
    }
    lock.unlock();
    
    if (objectList.empty())
        return NULL;

    // Add to result
    jobjectArray result = env->NewObjectArray(objectList.size(), classKeyFrame, NULL);
    int i = 0;
    for (std::list<jobject>::iterator it = objectList.begin(); it != objectList.end(); ++it) {
        env->SetObjectArrayElement(result, i++, *it);
    }

    // Release
    env->DeleteLocalRef(classKeyFrame);
    for (std::list<jobject>::iterator it = objectList.begin(); it != objectList.end(); ++it) {
        env->DeleteLocalRef(*it);
    }
    
    return result;
}

JNIEXPORT jint JNICALL
Java_com_tc_tar_TARNativeInterface_nativeGetKeyFrameCount(JNIEnv* env, jobject thiz) {
    assert (slamSystem != NULL);
    AndroidOutput3DWrapper* output = (AndroidOutput3DWrapper*)outputWrapper;
    return output->getKeyframesCount();
}

}
