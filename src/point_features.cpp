
//OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

//std
#include <iostream>
#include <cstdlib>
#include <vector>

//consts
const unsigned int MIN_NUM_FEATURES = 300; //minimum number of point fetaures

void orb_base(cv::Mat img, std::vector<cv::KeyPoint> keypoints,  cv::Ptr<cv::ORB> detector,cv::Mat descriptor){
  //clear previous points
  keypoints.clear();
  //detect and compute(extract) features
  //keypoints, descriptors	=	cv.Feature2D.detectAndCompute(	image, mask[, descriptors[, useProvidedKeypoints]]	)
  detector->detectAndCompute(img, cv::noArray(), keypoints, descriptor);
  //draw points on the image
  //cv::drawKeypoints (InputArray image, const std::vector< KeyPoint > &keypoints, InputOutputArray outImage, const Scalar &color=Scalar::all(-1), int flags=DrawMatchesFlags::DEFAULT)
  cv::drawKeypoints(img, keypoints, img, 255, cv::DrawMatchesFlags::DEFAULT );

}

void orb_base_mask(cv::Mat img, std::vector<cv::KeyPoint> keypoints,  cv::Ptr<cv::ORB> detector, cv::Mat descriptor,int x, int y, int width, int height){
  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);  // type of mask is CV_8U
  cv::Rect rect(x, y, width, height); //defining the rectangle where roi will be placed
  cv::Mat roi(mask, rect); //defining region of interest
  cv::rectangle(img, rect, cv::Scalar(0, 255, 0));  //rectangle(imageORBmask, 10, 10, 255, int thickness=1, int lineType=8, int shift=0)¶ //drawing green rectangle
  roi = cv::Scalar(255);  // roi is a sub-image of mask specified by cv::Rect object // we set elements in roi region of the mask to 255
  detector->detectAndCompute(img,mask,keypoints, descriptor);
  cv::drawKeypoints(img,keypoints,img,255,cv::DrawMatchesFlags::DEFAULT );
}

//**************keypoint Detectors ***************/

void FastFeatureDetector(cv::Mat img, std::vector<cv::KeyPoint> keypoints){
      cv::Ptr<cv::FastFeatureDetector> Fast = cv::FastFeatureDetector::create();
      Fast->detect(img, keypoints); //detect keypoints
      cv::drawKeypoints(img,keypoints,img,255,cv::DrawMatchesFlags::DEFAULT );

}

  void BriskDetector(cv::Mat img,std::vector<cv::KeyPoint> keypoints, cv::Mat descriptor ){
      cv::Ptr<cv::BRISK> BRISK = cv::BRISK::create();
      //BRISK->detect(img, keypoints);
      BRISK->detectAndCompute(img,cv::noArray(),keypoints,descriptor);
      cv::drawKeypoints(img,keypoints,img,255,cv::DrawMatchesFlags::DEFAULT );

  }

int main(int argc, char *argv[])
{
    cv::VideoCapture camera; //OpenCV video capture object
    cv::Mat image; //OpenCV image object
	  int cam_id; //camera id . Associated to device number in /dev/videoX
    cv::Ptr<cv::ORB> orb_detector = cv::ORB::create(); //ORB point feature detector
    orb_detector->setMaxFeatures(MIN_NUM_FEATURES);
    std::vector<cv::KeyPoint> point_set; //set of point features
    cv::Mat descriptor_set; //set of descriptors, for each feature there is an associated descriptor


	//check user args
	switch(argc)
	{
		case 1: //no argument provided, so try /dev/video0
			cam_id = 0;
			break;
		case 2: //an argument is provided. Get it and set cam_id
			cam_id = atoi(argv[1]);
			break;
		default:
			std::cout << "Invalid number of arguments. Call program as: webcam_capture [video_device_id]. " << std::endl;
			std::cout << "EXIT program." << std::endl;
			break;
	}

	//advertising to the user
	std::cout << "Opening video device " << cam_id << std::endl;

    //open the video stream and make sure it's opened
    if( !camera.open(cam_id) )
	{
        std::cout << "Error opening the camera. May be invalid device id. EXIT program." << std::endl;
        return -1;
    }

    //Process loop. Capture and point feature extraction. User can quit pressing a key
    while(1)
	{
		//Read image and check it. Blocking call up to a new image arrives from camera.
        if(!camera.read(image))
		{
            std::cout << "No image" << std::endl;
            cv::waitKey();
        }

    //****define more images***//
    cv::Mat imageORB;
    image.copyTo(imageORB);

    cv::Mat imageORBmask;
    image.copyTo(imageORBmask);

    cv::Mat imageFFD;
    image.copyTo(imageFFD);

    cv::Mat imageSBD;
    image.copyTo(imageSBD);


    //**************** Find ORB point fetaures and descriptors ****************************

    orb_base(imageORB, point_set,  orb_detector,descriptor_set);

    //**************** ##Try to detect features more sparse over all the image, by using the mask:****************************

    orb_base_mask(imageORBmask, point_set, orb_detector, descriptor_set,10, 10, 400, 400);

    //**************** ##Fast feature detector of keypoints:****************************

    FastFeatureDetector(imageFFD, point_set);

    //**************** ##SimpleBriskDetector:****************************

    BriskDetector(imageSBD, point_set,descriptor_set);

    //**************** ##SimpleBlobDetector:****************************

    cv::Mat matArray[3] = {image,imageORB,imageORBmask};
    cv::hconcat(matArray,3,imageORB);
    cv::Mat mat2[3] = {image,imageFFD,imageSBD};
    cv::hconcat(mat2,3,imageFFD);
    cv::imshow("keypoints detector algoritms: original image, FAST, BRISK", imageFFD);
    cv::imshow("Original image, ORB feature detector, ORB featre detector with mask", imageORB);



		//Waits 30 millisecond to check if 'q' key has been pressed. If so, breaks the loop. Otherwise continues.
    	if( (unsigned char)(cv::waitKey(30) & 0xff) == 'q' ) break;
    }
}
