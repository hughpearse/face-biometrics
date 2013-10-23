/*
 * Title: Facial Landmark XY coordinate extractor.
 * Author: Hugh Pearse
 */

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <cstring>
#include <cmath>
#include <math.h>
#include <stdio.h>

#include "flandmark_detector.h"

#define PI 3.141592654;

using namespace std;

//Function to find Lenght of sides of triangle
double DistanceTwoPoints(double x1, double y1, double x2, double y2)
{
    double x, y, distance;
    x = x2 - x1;
    y = y2 - y1;
    distance = pow(x,2) + pow(y,2);
    distance = sqrt(distance);
    return distance;
}

//Function to find angle with Sine rule
double otherAngleFind(double biggerAngle, double largestDistance, double smallDistance)
{
    double otherAngle;
    otherAngle = smallDistance *sin(biggerAngle*3.14159265/180);
    otherAngle = otherAngle/largestDistance;
    otherAngle = asin(otherAngle)*180.0 / PI;
    return otherAngle;
}

//Function to find angle opposite to largest side of triangle
double BiggerAngleFind(double largestDistance, double smallDistanceOne, double smallDistanceTwo)
{
    double biggerAngle;
    biggerAngle =  pow(smallDistanceOne,2) + pow(smallDistanceTwo, 2) - pow(largestDistance,2);
    biggerAngle = fabs(biggerAngle/(2*smallDistanceOne*smallDistanceTwo));
    biggerAngle = acos(biggerAngle)* 180.0 / PI;
    return biggerAngle;
}

//Calculate angle of triangle given three coordinates c++ code
double TriangleAngleCalculation(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double dist1, dist2, dist3;
    double angle1, angle2, angle3;
    double total;

    int largestLength = 0;
    dist1 = DistanceTwoPoints(x1, y1, x2, y2);
    dist2 = DistanceTwoPoints(x2, y2, x3, y3);
    dist3 = DistanceTwoPoints(x1, y1, x3, y3);

    //cout<<"Distance"<<dist1<<"  "<<dist2<<"  "<<dist3;

    if(dist1>dist2 && dist1 > dist3)
    {
        //cout<<"dist1 is greater";
        angle1 = BiggerAngleFind(dist1, dist2, dist3);
        angle2 = otherAngleFind(angle1, dist1, dist2);
        angle3 = otherAngleFind(angle1, dist1, dist3);

        //angle2 = OtherAngleFind(angle1, dist1, dist2);

        total=angle1+angle2+angle3;

        if(total <180)
        {
            angle1 = 180 - angle1;
        }
    }
    else if(dist2 > dist3 && dist2 > dist1)
    {
      //  cout<<"dist2 is greater";
        angle2 = BiggerAngleFind(dist2, dist1, dist3);
        angle1 = otherAngleFind(angle2, dist2, dist1);
        angle3 = otherAngleFind(angle2, dist2, dist3);

        total=angle1+angle2+angle3;

        if(total <180)
        {
            angle2 = 180 - angle2;
        }
    }
    else
    {
    //    cout<<"dist3 is greater";
        angle3 = BiggerAngleFind(dist3, dist1, dist2);
        angle1 = otherAngleFind(angle3, dist3, dist2);
        angle2 = otherAngleFind(angle3, dist3, dist2);

        total=angle1+angle2+angle3;

        if(total <180)
        {
            angle3 = 180 - angle3;
        }
    }

    //cout<<endl<<"Angle Between First Point and Second Point = "<<angle3<<endl;
    //cout<<"Angle Between First Point and Third Point = "<<angle2<<endl;
    //cout<<"Angle Between Second Point and Third Point = "<<angle1<<endl;
    return angle2;
}

void calculateMetrics(double *landmarks){
    CvPoint  LeftInnerEye;
    CvPoint  RightInnerEye;
    CvPoint  LeftMouth;
    CvPoint  RightMouth;
    CvPoint  LeftOuterEye;
    CvPoint  RightOuterEye;
    CvPoint  Nose;

    //left does not mean left on the face, it means on the left of the graph of the person looking
    try{
        LeftInnerEye = cvPoint(double(landmarks[2]), -double(landmarks[2+1]));
        RightInnerEye = cvPoint(double(landmarks[4]), -double(landmarks[4+1]));
        LeftMouth = cvPoint(double(landmarks[6]), -double(landmarks[6+1]));
        RightMouth = cvPoint(double(landmarks[8]), -double(landmarks[8+1]));
        LeftOuterEye = cvPoint(double(landmarks[10]), -double(landmarks[10+1]));
        RightOuterEye = cvPoint(double(landmarks[12]), -double(landmarks[12+1]));
        Nose = cvPoint(double(landmarks[14]), -double(landmarks[14+1]));
    } catch(int e){
        cout << "ERROR: point parse error\n";
    }
    cout << std::fixed;
    cout.precision(0);

    //TriangleAngleCalculation(x1, y1, x2, y2, x3, y3);
    //cout << "Eye Nose Eye Angle: " << (double)TriangleAngleCalculation(LeftOuterEye.x, LeftOuterEye.y, Nose.x, Nose.y, RightOuterEye.x, RightOuterEye.y) << "\n";
    //cout << "Right Eye Nose Mouth Angle: " << (double)TriangleAngleCalculation(RightOuterEye.x, RightOuterEye.y, Nose.x, Nose.y, RightMouth.x, RightMouth.y) << "\n";
    //cout << "Mouth Nose Mouth Angle: " << (double)TriangleAngleCalculation(RightMouth.x, RightMouth.y, Nose.x, Nose.y, LeftMouth.x, LeftMouth.y) << "\n";
    //cout << "Left Eye Nose Mouth Angle: " << (double)TriangleAngleCalculation(LeftOuterEye.x, LeftOuterEye.y, Nose.x, Nose.y, LeftMouth.x, LeftMouth.y) << "\n";
    cout << "" << (double)TriangleAngleCalculation(LeftOuterEye.x, LeftOuterEye.y, Nose.x, Nose.y, RightOuterEye.x, RightOuterEye.y);
    cout << "," << (double)TriangleAngleCalculation(RightOuterEye.x, RightOuterEye.y, Nose.x, Nose.y, RightMouth.x, RightMouth.y);
    cout << "," << (double)TriangleAngleCalculation(RightMouth.x, RightMouth.y, Nose.x, Nose.y, LeftMouth.x, LeftMouth.y);
    cout << "," << (double)TriangleAngleCalculation(LeftOuterEye.x, LeftOuterEye.y, Nose.x, Nose.y, LeftMouth.x, LeftMouth.y);

    double LeftEyeSlope = ((double)(LeftInnerEye.y - LeftOuterEye.y) / (double)(LeftInnerEye.x - LeftOuterEye.x));
    double RightEyeSlope = ((double)(RightInnerEye.y - RightOuterEye.y) / (double)(RightInnerEye.x - RightOuterEye.x));
    double MouthSlope = ((double)(RightMouth.y - LeftMouth.y) / (double)(RightMouth.x - LeftMouth.x));
    double cosineEyeAngle;
    double trueEyeAngle;
    
    int pt1, pt2, pt3, pt4;

    if(LeftEyeSlope == RightEyeSlope){
        trueEyeAngle = 180;
        cout << "," << trueEyeAngle;
    } else {
        pt1 = LeftInnerEye.x - LeftOuterEye.x;
        pt2 = LeftInnerEye.y - LeftOuterEye.y;
        pt3 = RightInnerEye.x - RightOuterEye.x;
        pt4 = RightInnerEye.y - RightOuterEye.y;
        cosineEyeAngle = ( (pt1 * pt3) + (pt2 * pt4) )/((sqrt(pt1*pt1 + pt2*pt2)) * (sqrt(pt3*pt3 + pt4*pt4)) );
	trueEyeAngle = acos(cosineEyeAngle) * 180 / 3.141592654;
        //cout << "True eye angle intersecting eye: " << trueEyeAngle << "\n";
        cout << "," << trueEyeAngle;
    }

    if(LeftEyeSlope == MouthSlope){
        trueEyeAngle = 180;
        cout << "," << trueEyeAngle;
    } else {
        pt1 = LeftInnerEye.x - LeftOuterEye.x;
        pt2 = LeftInnerEye.y - LeftOuterEye.y;
        pt3 = Nose.x - (double)(Nose.x+1);
        pt4 = Nose.y - Nose.y;
        cosineEyeAngle = ( (pt1 * pt3) + (pt2 * pt4) )/((sqrt(pt1*pt1 + pt2*pt2)) * (sqrt(pt3*pt3 + pt4*pt4)) );
        trueEyeAngle = acos(cosineEyeAngle) * 180 / 3.141592654;
        //cout << "True left eye angle intersecting nose: " << trueEyeAngle << "\n";
        cout << "," << trueEyeAngle;
    }

    if(RightEyeSlope == MouthSlope){
        trueEyeAngle = 180;
        cout << "," << trueEyeAngle;
    } else {
        pt1 = RightOuterEye.x - RightInnerEye.x;
        pt2 = RightOuterEye.y - RightInnerEye.y;
        pt3 = Nose.x - (double)(Nose.x+1);
        pt4 = Nose.y - Nose.y;
        cosineEyeAngle = ( (pt1 * pt3) + (pt2 * pt4) )/((sqrt(pt1*pt1 + pt2*pt2)) * (sqrt(pt3*pt3 + pt4*pt4)) );
        trueEyeAngle = acos(cosineEyeAngle) * 180 / 3.141592654;
        //cout << "True right eye angle intersecting nose: " << trueEyeAngle << "\n";
        cout << "," << trueEyeAngle;
    }

        
    //cout << "Distance between nose and average eye height: " << DistanceTwoPoints( Nose.x, Nose.y, Nose.x, (double)((LeftOuterEye.y + RightOuterEye.y)/2) ) << "\n";
    //cout << "Distance between outer eyes: " << DistanceTwoPoints(LeftOuterEye.x, LeftOuterEye.y, RightOuterEye.x, RightOuterEye.y) << "\n";
    //cout << "Distance between inner eyes: " << DistanceTwoPoints(LeftInnerEye.x, LeftInnerEye.y, RightInnerEye.x, RightInnerEye.y) << "\n";
    cout << "," << DistanceTwoPoints( Nose.x, Nose.y, Nose.x, (double)((LeftOuterEye.y + RightOuterEye.y)/2) );
    cout << "," << DistanceTwoPoints(LeftOuterEye.x, LeftOuterEye.y, RightOuterEye.x, RightOuterEye.y);
    cout << "," << DistanceTwoPoints(LeftInnerEye.x, LeftInnerEye.y, RightInnerEye.x, RightInnerEye.y) << "\n";
}

void detectFaceInImage(IplImage *orig, IplImage* input, CvHaarClassifierCascade* cascade, FLANDMARK_Model *model, int *bbox, double *landmarks)
{
    // Smallest face size.
    CvSize minFeatureSize = cvSize(40, 40);
    int flags =  CV_HAAR_DO_CANNY_PRUNING;
    // How detailed should the search be.
    float search_scale_factor = 1.1f;
    CvMemStorage* storage;
    CvSeq* rects;
    int nFaces;

    storage = cvCreateMemStorage(0);
    cvClearMemStorage(storage);

    // Detect all the faces in the greyscale image.
    rects = cvHaarDetectObjects(input, cascade, storage, search_scale_factor, 2, flags, minFeatureSize);
    nFaces = rects->total;

    double t = (double)cvGetTickCount();
    for (int iface = 0; iface < (rects ? nFaces : 0); ++iface)
    {
        CvRect *r = (CvRect*)cvGetSeqElem(rects, iface);
        
        bbox[0] = r->x;
        bbox[1] = r->y;
        bbox[2] = r->x + r->width;
        bbox[3] = r->y + r->height;
        
        flandmark_detect(input, bbox, model, landmarks);

        // display landmarks
        cvRectangle(orig, cvPoint(bbox[0], bbox[1]), cvPoint(bbox[2], bbox[3]), CV_RGB(255,0,0) );
        cvRectangle(orig, cvPoint(model->bb[0], model->bb[1]), cvPoint(model->bb[2], model->bb[3]), CV_RGB(0,0,255) );
        cvCircle(orig, cvPoint((int)landmarks[0], (int)landmarks[1]), 3, CV_RGB(0, 0,255), CV_FILLED);

        //i starts at 2 so that only one point for the nose is output
        for (int i = 2; i < 2*model->data.options.M; i += 2)
        {
            cvCircle(orig, cvPoint(int(landmarks[i]), int(landmarks[i+1])), 3, CV_RGB(255,0,0), CV_FILLED);
            //CvPoint  P1 = cvPoint(int(landmarks[i]), int(landmarks[i+1]));
            //cout << "" << P1.x << ",-" << P1.y << "" << "\n";
        }
	calculateMetrics(landmarks);
    }
    t = (double)cvGetTickCount() - t;
    int ms = cvRound( t / ((double)cvGetTickFrequency() * 1000.0) );

    if (nFaces > 0)
    {
        //printf("Faces detected: %d; Detection of facial landmark on all faces took %d ms\n", nFaces, ms);
    } else {
        printf("NO Face\n");
    }
    
    cvReleaseMemStorage(&storage);
}

int main( int argc, char** argv ) 
{
    char flandmark_window[] = "flandmark_example1";
    double t;
    int ms;
    
    if (argc < 2)
    {
      fprintf(stderr, "Usage: flandmark_1 <path_to_input_image> [<path_to_output_image>]\n");
      exit(1);
    }
    
    cvNamedWindow(flandmark_window, 0);
    
    // Haar Cascade file, used for Face Detection.
    char faceCascadeFilename[] = "haarcascade_frontalface_alt.xml";
    // Load the HaarCascade classifier for face detection.
    CvHaarClassifierCascade* faceCascade;
    faceCascade = (CvHaarClassifierCascade*)cvLoad(faceCascadeFilename, 0, 0, 0);
    if( !faceCascade )
    {
        printf("Couldnt load Face detector '%s'\n", faceCascadeFilename);
        exit(1);
    }

     // ------------- begin flandmark load model
    t = (double)cvGetTickCount();
    FLANDMARK_Model * model = flandmark_init("flandmark_model.dat");

    if (model == 0)
    {
        printf("Structure model wasn't created. Corrupted file flandmark_model.dat?\n");
        exit(1);
    }

    t = (double)cvGetTickCount() - t;
    ms = cvRound( t / ((double)cvGetTickFrequency() * 1000.0) );
    //printf("Structure model loaded in %d ms.\n", ms);
    // ------------- end flandmark load model
    
    // input image
    IplImage *frame = cvLoadImage(argv[1]);
    if (frame == NULL)
    {
      fprintf(stderr, "Cannot open image %s. Exiting...\n", argv[1]);
      exit(1);
    }
    // convert image to grayscale
    IplImage *frame_bw = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
    cvConvertImage(frame, frame_bw);
    
    int *bbox = (int*)malloc(4*sizeof(int));
    double *landmarks = (double*)malloc(2*model->data.options.M*sizeof(double));
    detectFaceInImage(frame, frame_bw, faceCascade, model, bbox, landmarks);

    //cvShowImage(flandmark_window, frame);
    //cvWaitKey(0);
    
    if (argc == 3)
    {
      printf("Saving image to file %s...\n", argv[2]);
      cvSaveImage(argv[2], frame);
    }
    
    // cleanup
    free(bbox);
    free(landmarks);
    cvDestroyWindow(flandmark_window);
    cvReleaseImage(&frame);
    cvReleaseImage(&frame_bw);
    cvReleaseHaarClassifierCascade(&faceCascade);
    flandmark_free(model);
}
