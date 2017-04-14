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
#include "ImageSource.h"
#include "Android/FilesImageSource.h"


// FIXME: remove hard code
#define IMAGE_DIR "/sdcard/LSD/images"

using namespace lsd_slam;
boost::thread *gLsdThread = NULL;
#if 0
ThreadMutexObject<bool> loopDone(false);
std::vector<std::string> files;
int w, h, w_inp, h_inp;
Sophus::Matrix3f K;
Undistorter* gUndistorter = NULL;
SlamSystem * gSlamSystem = NULL;
#endif
Output3DWrapper* gOutputWrapper = NULL;
ImageSource* gImageSource = NULL;

#if 0
void Loop() {
    LOGD("Loop start\n");
    assert (gSlamSystem != NULL);
    assert (gOutputWrapper != NULL);
    assert (gImageSource != NULL);
    
    // get HZ
    double hz = 30;
    cv::Mat image = cv::Mat(h, w, CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; i < files.size(); i++)
    {
        if(loopDone.getValue())
            break;

        cv::Mat imageDist = cv::Mat(h, w, CV_8U);
        imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
        if(imageDist.rows != h_inp || imageDist.cols != w_inp)
        {
            if(imageDist.rows * imageDist.cols == 0)
                LOGE("failed to load image %s! skipping.\n", files[i].c_str());
            else
                LOGE("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                        files[i].c_str(), w, h, imageDist.cols, imageDist.rows);
            continue;
        }

        assert(imageDist.type() == CV_8U);
        gUndistorter->undistort(imageDist, image);

        assert(image.type() == CV_8U);
#if 0        
        if(runningIDX == 0)
        {
            gSlamSystem->randomInit(image.data, fakeTimeStamp, runningIDX);
        }
        else
        {
            gSlamSystem->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp);
        }
#endif

        while (!(gImageSource->getBuffer()->size() > 0)) continue;
        TimestampedMat* image = gImageSource->getBuffer()->first();
	    gImageSource->getBuffer()->popFront();
	    if(runningIDX == 0)
        {
            gSlamSystem->randomInit(image->data.data, fakeTimeStamp, runningIDX);
        }
        else
        {
            gSlamSystem->trackFrame(image->data.data, runningIDX, hz == 0, fakeTimeStamp);
        }
        
        //printTrans(gSlamSystem->getCurrentPoseEstimateScale().matrix());

        runningIDX++;
        fakeTimeStamp+=0.03;
 
        if(fullResetRequested)
        {
            LOGD("FULL RESET!\n");
            delete gSlamSystem;
            gSlamSystem = new SlamSystem(w, h, K, doSlam);
            gSlamSystem->setVisualization(gOutputWrapper);
            fullResetRequested = false;
            runningIDX = 0;
        }
    }
    LOGD("Loop done.\n");
}
#endif

class LsdSlamWrapper : public Notifiable {
public:
    LsdSlamWrapper(ImageSource* source, Output3DWrapper* output) : loopDone_(false) {
        assert (source != NULL);
        assert (output != NULL);
        imageSource_ = source;
        imageSource_->getBuffer()->setReceiver(this);
        // make slam system
    	slamSystem_ = new SlamSystem(source->width(), source->height(), source->K(), doSlam);
    	slamSystem_->setVisualization(output);

    	Resolution::getInstance(source->width(), source->height());
	    Intrinsics::getInstance(source->fx(), source->fy(), source->cx(), source->cy());
    }

    SlamSystem* getSlamSystem() const {
        return slamSystem_;
    }

    void stop() {
        loopDone_.assignValue(true);
    }
    
    void Loop() {
        LOGD("Loop start");
        int runningIDX = 0;
        while (!loopDone_.getValue()) {
            boost::unique_lock<boost::recursive_mutex> waitLock(imageSource_->getBuffer()->getMutex());
    		while (!(imageSource_->getBuffer()->size() > 0)) {
    		    notifyCondition.wait(waitLock);
    		}
    		waitLock.unlock();

            TimestampedMat* image = imageSource_->getBuffer()->first();
		    imageSource_->getBuffer()->popFront();
            if(runningIDX == 0){
                slamSystem_->randomInit(image->data.data, image->timestamp.toSec(), runningIDX);
            } else {
                slamSystem_->trackFrame(image->data.data, runningIDX, false, image->timestamp.toSec());
            }
            runningIDX++;

            delete image;
        }
        LOGD("Loop exit.");
    }
private:
    ImageSource* imageSource_;
    SlamSystem* slamSystem_;
    ThreadMutexObject<bool> loopDone_;
};
LsdSlamWrapper* gLsdSlam = NULL;


extern "C"{
JavaVM* jvm = NULL;

//init LSD
JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeInit(JNIEnv* env, jobject thiz, jstring calibPath) {
	LOGD("nativeInit");
    //init jni
	env->GetJavaVM(&jvm);

	const char *calibFile = env->GetStringUTFChars(calibPath, 0);
	LOGD("calibFile: %s\n", calibFile);
	gImageSource = new FilesImageSource(IMAGE_DIR);
	gImageSource->setCalibration(calibFile);
//	gUndistorter = Undistorter::getUndistorterForFile(calibFile);
	env->ReleaseStringUTFChars(calibPath, calibFile);  // release resources
#if 0
	if(gUndistorter == NULL) {
		LOGE("need camera calibration file! (set using -c FILE)\n");
		exit(0);
	}

	w = gUndistorter->getOutputWidth();
	h = gUndistorter->getOutputHeight();

	w_inp = gUndistorter->getInputWidth();
	h_inp = gUndistorter->getInputHeight();
	LOGD("w=%d, h=%d, w_inp=%d, h_inp=%d\n", w, h, w_inp, h_inp);

	float fx = gUndistorter->getK().at<double>(0, 0);
	float fy = gUndistorter->getK().at<double>(1, 1);
	float cx = gUndistorter->getK().at<double>(2, 0);
	float cy = gUndistorter->getK().at<double>(2, 1);
	
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
	LOGD("fx=%f, fy=%f, cx=%f, cy=%f\n", fx, fy, cx, cy);
#endif

#if 0
    int w = gImageSource->width();
    int h = gImageSource->height();
    float fx = gImageSource->fx();
    float fy = gImageSource->fy();
    float cx = gImageSource->cx();
    float cy = gImageSource->cy();
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    LOGD("fx=%f, fy=%f, cx=%f, cy=%f\n", fx, fy, cx, cy);

	Resolution::getInstance(w, h);
	Intrinsics::getInstance(fx, fy, cx, cy);
#endif

	gOutputWrapper = new AndroidOutput3DWrapper(gImageSource->width(), gImageSource->height());
	
#if 0
	// make slam system
	gSlamSystem = new SlamSystem(w, h, K, doSlam);
	gSlamSystem->setVisualization(gOutputWrapper);

    // open image files: first try to open as file.
	std::string source = IMAGE_DIR;

    if(getdir(source, files) >= 0) {
        LOGD("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
    }
    else if(getFile(source, files) >= 0) {
        LOGD("found %d image files in file %s!\n", (int)files.size(), source.c_str());
    }
    else {
        LOGD("could not load file list! wrong path / file?\n");
    }
#endif

    gLsdSlam = new LsdSlamWrapper(gImageSource, gOutputWrapper);
    boost::function0< void > f =  boost::bind(&LsdSlamWrapper::Loop, gLsdSlam);
    boost::thread thread(f);
}

// clean up
JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeDestroy(JNIEnv* env, jobject thiz) {
	LOGD("nativeDestroy\n");
	gLsdSlam->stop();
#if 0
	loopDone.assignValue(true);
	if (gLsdThread != NULL) {
    	gLsdThread->join();
	}
#endif
	LOGD("nativeDestroy done.\n");
}

JNIEXPORT void JNICALL
Java_com_tc_tar_TARNativeInterface_nativeStart(JNIEnv* env, jobject thiz) {
	LOGD("nativeStart\n");
	gImageSource->run();
//	gLsdThread = new boost::thread(Loop);
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

    Sophus::Matrix4f m = gLsdSlam->getSlamSystem()->getCurrentPoseEstimateScale().matrix();
    GLfloat* pose = m.data();
    jfloat array1[length];
    memcpy(array1, pose, sizeof(jfloat) * length);

    env->SetFloatArrayRegion(result, 0, length, array1);
    return result;
}

JNIEXPORT jobjectArray JNICALL
Java_com_tc_tar_TARNativeInterface_nativeGetAllKeyFrames(JNIEnv* env, jobject thiz) {
    //LOGD("nativeGetAllKeyFrames\n");
    assert (gOutputWrapper != NULL);
    AndroidOutput3DWrapper* output = (AndroidOutput3DWrapper*)gOutputWrapper;
    ThreadMutexObject<std::map<int, Keyframe *> >& keyframes = output->getKeyframes();
    
    jclass classKeyFrame = env->FindClass("com/tc/tar/LSDKeyFrame");
    std::list<jobject> objectList;

    boost::mutex::scoped_lock lock(keyframes.getMutex());
#if 0
    static bool hasDumped = false;
    if (keyframes.getReference().size() > 20 && !hasDumped) {
        dumpCloudPoint(keyframes.getReference());
        hasDumped = true;
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
    assert (gOutputWrapper != NULL);
    AndroidOutput3DWrapper* output = (AndroidOutput3DWrapper*)gOutputWrapper;
    return output->getKeyframesCount();
}

/**
    Get current image data
    format: 0 means ARGB
*/
JNIEXPORT jbyteArray JNICALL
Java_com_tc_tar_TARNativeInterface_nativeGetCurrentImage(JNIEnv* env, jobject thiz, jint format) {
    assert (gOutputWrapper != NULL);
    AndroidOutput3DWrapper* output = (AndroidOutput3DWrapper*)gOutputWrapper;
    ThreadMutexObject<unsigned char* >& image = output->getImageBuffer();
    if (image.getReference() == NULL)
        return NULL;

    boost::mutex::scoped_lock lock(image.getMutex());
    int originSize = output->getImageBufferSize();
    const unsigned char* originData = image.getReference();
    int imgSize = originSize / 3 * 4;
    unsigned char* imgData = new unsigned char[imgSize];
    for (int i = 0; i < originSize / 3; ++i) {
        imgData[i * 4] = originData[i * 3];
        imgData[i * 4 + 1] = originData[i * 3 + 1];
        imgData[i * 4 + 2] = originData[i * 3 + 2];
        imgData[i * 4 + 3] = (unsigned char)0xff;
    }
    lock.unlock();

    jbyteArray byteArray = env->NewByteArray(imgSize);
    env->SetByteArrayRegion(byteArray, 0, originSize, (jbyte*)imgData);
    
    delete imgData;
    return byteArray;
}

}
