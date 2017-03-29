#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
cv::Mat imgOld;
cv::Mat image;
cv::Mat image3;
cv::Mat image4;
cv::Mat img2;
cv::Mat invSrc;
bool bol=false;
int i=0;
void MyPolygon( cv::Mat img )
{
    int lineType = 8;
    /* Create some points */
    cv::Point rook_points[1][28];
    rook_points[0][0] = cv::Point( 0, 480 );
    rook_points[0][1] = cv::Point( 0, 468 );
    rook_points[0][2] = cv::Point( 13, 455 );
    rook_points[0][3] = cv::Point( 14, 450 );
    //rook_points[0][3] = cv::Point( 115, 390 );
    //rook_points[0][] = cv::Point( 80, 405 );

    //rook_points[0][4] = cv::Point( 14, 455 );
    rook_points[0][4] = cv::Point( 43, 420 );
    rook_points[0][5] = cv::Point( 109, 390 );
    rook_points[0][6] = cv::Point( 141, 381 );
    rook_points[0][7] = cv::Point( 186, 375 );
    rook_points[0][8] = cv::Point( 184, 360 );
    rook_points[0][9] = cv::Point( 216, 360 );
    rook_points[0][10] = cv::Point( 216, 343 );
    rook_points[0][11] = cv::Point( 216, 343 );
    rook_points[0][12] = cv::Point( 233, 343 );
    rook_points[0][13] = cv::Point( 234, 357 );
    rook_points[0][14] = cv::Point( 253, 357 );
    rook_points[0][15] = cv::Point( 253, 357 );
    rook_points[0][16] = cv::Point( 254, 369 );
    rook_points[0][17] = cv::Point( 254, 369 );
    rook_points[0][18] = cv::Point( 410, 370 );
    rook_points[0][19] = cv::Point( 410, 370 );
    rook_points[0][20] = cv::Point( 423, 350 );
    rook_points[0][21] = cv::Point( 439, 359 );
    rook_points[0][22] = cv::Point( 473, 357 );
    rook_points[0][23 ] = cv::Point( 486, 376 );
    rook_points[0][24 ] = cv::Point( 546, 383 );
    rook_points[0][25 ] = cv::Point( 640, 423 );
    rook_points[0][26 ] = cv::Point( 640, 480 );

    /*114  381
    rook_points[0][24 ] = cv::Point( 640, 480 );
    
    rook_points[0][2] = cv::Point( 640, 468 );
    rook_points[0][3] = cv::Point( 640, 480 );

    rook_points[0][2] = cv::Point( 3*640/4.0, 13*640/16.0 );
    rook_points[0][3] = cv::Point( 11*640/16.0, 13*640/16.0 );
    rook_points[0][4] = cv::Point( 19*640/32.0, 3*640/8.0 );
    rook_points[0][5] = cv::Point( 3*640/4.0, 3*640/8.0 );
    rook_points[0][6] = cv::Point( 3*640/4.0, 640/8.0 );
    rook_points[0][7] = cv::Point( 26*640/40.0, 640/8.0 );
    rook_points[0][8] = cv::Point( 26*640/40.0, 640/4.0 );
    rook_points[0][9] = cv::Point( 22*640/40.0, 640/4.0 );
    rook_points[0][10] = cv::Point( 22*640/40.0, 640/8.0 );
    rook_points[0][11] = cv::Point( 18*640/40.0, 640/8.0 );
    rook_points[0][12] = cv::Point( 18*640/40.0, 640/4.0 );
    rook_points[0][13] = cv::Point( 14*640/40.0, 640/4.0 );
    rook_points[0][14] = cv::Point( 14*640/40.0, 640/8.0 );
    rook_points[0][15] = cv::Point( 640/4.0, 640/8.0 );
    rook_points[0][16] = cv::Point( 640/4.0, 3*640/8.0 );
    rook_points[0][17] = cv::Point( 13*640/32.0, 3*640/8.0 );
    rook_points[0][18] = cv::Point( 5*640/16.0, 13*640/16.0 );
    rook_points[0][19] = cv::Point( 640/4.0, 13*640/16.0) ;
    */
    const cv::Point* ppt[1] = { rook_points[0] };
    int npt[] = { 27 };
    fillPoly( img,
              ppt,
              npt,
                  1,
              cv::Scalar( 0, 0, 0 ),
              lineType );
    //cv::imshow("view", img);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    /*
    img =  cv_bridge::toCvShare(msg, "bgr8")->image;
        std::vector<cv::Mat> ch;

    cv::split(img,ch);
    //get image only the range 0 to 85
    cv::inRange(img,cv::Scalar(100,100, 100), cv::Scalar(255, 255, 255), img);
    //cv::inRange(ch[1],cv::Scalar(0, 100, 0), cv::Scalar(255, 200, 255), ch[1]);

    //cv::inRange(ch[0],cv::Scalar(0, 100, 0), cv::Scalar(255, 200, 255), ch[0]);


    //cv::merge(ch,img);
    cv::threshold(img,img, 101, 255, CV_THRESH_BINARY);
      cv::imshow("view", img);

       char str[10];

      sprintf(str, "str%d.png", i);
      i++;
      cv::imwrite(str,img);
    */char str[10];

    //cv::split(img,ch);

    //get image only the range 0 to 85
    image =  cv_bridge::toCvShare(msg, "16UC1")->image;
          sprintf(str, "str%d.png", i);
      //i++;
      //cv::imwrite(str,image);
    img2 =  cv_bridge::toCvShare(msg, "32FC1")->image;
    image4 =  cv_bridge::toCvShare(msg, "16UC1")->image;

    //Mat floatMat(640, 480, "CV_32FC1");
cv::Mat ucharMat, ucharMatScaled;
//img2.convertTo(ucharMat, "CV_8UC1");

// scale values from 0..1 to 0..255
img2.convertTo(ucharMatScaled, CV_8UC1, 255, 0); 
   // cv::inRange(image,cv::Scalar(100,100, 100), cv::Scalar(255, 255, 255), image);

    //cv::inRange(ch[1],cv::Scalar(0, 100, 0), cv::Scalar(255, 200, 255), ch[1]);

    //cv::inRange(ch[0],cv::Scalar(0, 100, 0), cv::Scalar(255, 200, 255), ch[0]);

        //filtersdouble min;
    double max;
    double min;
    cv::minMaxIdx(image, &min, &max);
    cv::Mat adjMap;
    image.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 

    cv::Mat falseColorsMap;

    cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_OCEAN);
    /*
    cv::Mat kernel;
    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    
    cv::dilate(image, image, kernel);
    //cv::morphologyEx(image,image, cv::MORPH_OPEN, kernel);
    cv::erode(image, image, kernel);
    cv::erode(image, image, kernel);
    //cv::morphologyEx(ch[2], ch[2], cv::MORPH_OPEN, kernel);
    //cv::merge(ch,img);
    cv::threshold(image,image, 101, 255, CV_THRESH_BINARY_INV);
    */
    //MyPolygon(image);
    // cv::rectangle( image, cv::Point( 0, 0 ),cv::Point( 640, 250),cv::Scalar(0, 0, 0 ),CV_FILLED, CV_AA );
//     cv::threshold(image,image, 101, 255, CV_THRESH_BINARY);

    cv::Mat kernel;
    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    //cv::dilate(image, image, kernel);
    //cv::morphologyEx(image,image, cv::MORPH_OPEN, kernel);
//    cv::erode(image, image, kernel);
//cv::erode(image, image, kernel);
//cv::erode(image, image, kernel);
//    cv::dilate(image, image, kernel);
    // Canny algorithm
//    cv::dilate(image, image, kernel);
/*
    cv::Mat contours, cdst;
    Canny(image,contours,50,350);
    cv::Mat contoursInv;
    int houghVote;
    double PI = 3.1415;
    //threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

    //std::vector<Vec2f> lines;
    cv::vector<cv::Vec4i> lines;

  HoughLinesP(contours, lines, 1, CV_PI/180, 10, 5, 3 );
  cvtColor(contours, cdst, CV_GRAY2BGR);
  for( size_t i = 0; i < lines.size(); i++ )
  {
    cv::Vec4i l = lines[i];
    double m = ((double)l[3]-(double)l[1])/((double)l[2]-(double)l[0]); //  y2-y1/x2-x1
    if(m>-.3 && m<.3)
        line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
  }*/

//Below is the result of the Hough transform.

//namedWindow("Display Image1", WINDOW_AUTOSIZE );
//    imshow("Display Image1", contours);


    cv::Mat output;
    cv::Point2f inputQuad[4];
    // Output Quadilateral or World plane coordinates
    cv::Point2f outputQuad[4];
    cv::Mat lambda( 2, 4, CV_32FC1 );
    lambda = cv::Mat::zeros( image.rows, image.cols, image.type() );
    //Mat src2=imread(argv[2],1);
    //namedWindow("src",1);imshow("src",src);

 // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input
    inputQuad[0] = cv::Point2f( 0,200 );
    inputQuad[1] = cv::Point2f( 640 ,200);
    inputQuad[2] = cv::Point2f( 640,350);
    inputQuad[3] = cv::Point2f( 0,350  );
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = cv::Point2f( 0,0 );
    outputQuad[1] = cv::Point2f( image.cols-1,0);
    outputQuad[2] = cv::Point2f( image.cols-1,image.rows-1);
    outputQuad[3] = cv::Point2f( 0,image.rows-1  );

    // Get the Perspective Transform Matrix i.e. lambda
    lambda = cv::getPerspectiveTransform( inputQuad, outputQuad );
    // Apply the Perspective Transform just found to the src image
    cv::warpPerspective(image,output,lambda,output.size() );

    //namedWindow("src",1);imshow("src",output);
    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image", cdst);
   
   //bitwise_and(src2,src2,fin_img,fin_img); 
     invSrc =  cv::Scalar::all(255)  -img2 ;

   
	invSrc.convertTo(invSrc, CV_8UC1, 255, 0); 
//     bitwise_and(img2,img2,invSrc,invSrc);
    //cv::cvtColor(image,image,CV_GRAY2BGR);

  //  double alpha = 0.5; double beta; double input;
  //  beta = ( 1.0 - alpha );
  //cv::addWeighted( image, alpha, falseColorsMap, beta, 0.0, image);

    cv::imshow("view", ucharMatScaled);
    cv::imshow("view2",invSrc);
    //waitKey(0);


  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    bol =false;
  }
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    /*
    img =  cv_bridge::toCvShare(msg, "bgr8")->image;
        std::vector<cv::Mat> ch;

    cv::split(img,ch);
    //get image only the range 0 to 85
    cv::inRange(img,cv::Scalar(100,100, 100), cv::Scalar(255, 255, 255), img);
    //cv::inRange(ch[1],cv::Scalar(0, 100, 0), cv::Scalar(255, 200, 255), ch[1]);

    //cv::inRange(ch[0],cv::Scalar(0, 100, 0), cv::Scalar(255, 200, 255), ch[0]);


    //cv::merge(ch,img);
    cv::threshold(img,img, 101, 255, CV_THRESH_BINARY);
      cv::imshow("view", img);

       char str[10];

      sprintf(str, "str%d.png", i);
      i++;
      cv::imwrite(str,img);
    */char str[10];

    //cv::split(img,ch);

    //get image only the range 0 to 85
    image =  cv_bridge::toCvShare(msg, "bgr8")->image;
    image3 =  cv_bridge::toCvShare(msg, "bgr8")->image;
          sprintf(str, "str%d.png", i);
      i++;
      //cv::imwrite(str,image);
    img2 =  cv_bridge::toCvShare(msg, "bgr8")->image;
   // cv::inRange(image,cv::Scalar(100,100, 100), cv::Scalar(255, 255, 255), image);

    //cv::inRange(ch[1],cv::Scalar(0, 100, 0), cv::Scalar(255, 200, 255), ch[1]);

    //cv::inRange(ch[0],cv::Scalar(0, 100, 0), cv::Scalar(255, 200, 255), ch[0]);

        //filters
    /*
    cv::Mat kernel;
    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));

    cv::dilate(image, image, kernel);
    //cv::morphologyEx(image,image, cv::MORPH_OPEN, kernel);
    cv::erode(image, image, kernel);
    cv::erode(image, image, kernel);
    //cv::morphologyEx(ch[2], ch[2], cv::MORPH_OPEN, kernel);
    //cv::merge(ch,img);
    cv::threshold(image,image, 101, 255, CV_THRESH_BINARY_INV);
    */
    MyPolygon(image);
     cv::rectangle( image, cv::Point( 0, 0 ),cv::Point( 640, 250),cv::Scalar(0, 0, 0 ),CV_FILLED,
CV_AA );
//     cv::threshold(image,image, 101, 255, CV_THRESH_BINARY);

    cv::Mat kernel;
    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    //cv::dilate(image, image, kernel);
    //cv::morphologyEx(image,image, cv::MORPH_OPEN, kernel);
//    cv::erode(image, image, kernel);
//cv::erode(image, image, kernel);
//cv::erode(image, image, kernel);
//    cv::dilate(image, image, kernel);
    // Canny algorithm
//    cv::dilate(image, image, kernel);
/*
    cv::Mat contours, cdst;
    Canny(image,contours,50,350);
    cv::Mat contoursInv;
    int houghVote;
    double PI = 3.1415;
    //threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

    //std::vector<Vec2f> lines;
    cv::vector<cv::Vec4i> lines;

  HoughLinesP(contours, lines, 1, CV_PI/180, 10, 5, 3 );
  cvtColor(contours, cdst, CV_GRAY2BGR);
  for( size_t i = 0; i < lines.size(); i++ )
  {
    cv::Vec4i l = lines[i];
    double m = ((double)l[3]-(double)l[1])/((double)l[2]-(double)l[0]); //  y2-y1/x2-x1
    if(m>-.3 && m<.3)
        line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
  }*/

//Below is the result of the Hough transform.

//namedWindow("Display Image1", WINDOW_AUTOSIZE );
//    imshow("Display Image1", contours);


    cv::Mat output;
    cv::Point2f inputQuad[4];
    // Output Quadilateral or World plane coordinates
    cv::Point2f outputQuad[4];
    cv::Mat lambda( 2, 4, CV_32FC1 );
    lambda = cv::Mat::zeros( image.rows, image.cols, image.type() );
    //Mat src2=imread(argv[2],1);
    //namedWindow("src",1);imshow("src",src);

 // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input
    inputQuad[0] = cv::Point2f( 0,200 );
    inputQuad[1] = cv::Point2f( 640 ,200);
    inputQuad[2] = cv::Point2f( 640,350);
    inputQuad[3] = cv::Point2f( 0,350  );
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = cv::Point2f( 0,0 );
    outputQuad[1] = cv::Point2f( image.cols-1,0);
    outputQuad[2] = cv::Point2f( image.cols-1,image.rows-1);
    outputQuad[3] = cv::Point2f( 0,image.rows-1  );

    // Get the Perspective Transform Matrix i.e. lambda
    lambda = cv::getPerspectiveTransform( inputQuad, outputQuad );
    // Apply the Perspective Transform just found to the src image
    cv::warpPerspective(image,output,lambda,output.size() );
    cv::Mat  fin_img;
    cv::vector<cv::Mat> channels;
    cv::vector<cv::Mat> rgbChannels(3);
    cv::split(invSrc, rgbChannels);
   /*channels.push_back(

                      (rgbChannels[0] & invSrc) |
                      (rgbChannels[1] & invSrc) |
                      (rgbChannels[2] & invSrc) 
                      );
  /*
    cv::merge(channels, fin_img);
*/
    //cv::bitwise_and(image3, image3, rgbChannels[0],rgbChannels[0]);
    //namedWindow("src",1);imshow("src",output);
    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image", cdst);
    //cv::imshow("view22",rgbChannels[0]);
    //waitKey(0);

bitwise_and(image3,image3,invSrc,invSrc);
    cv::imshow("view02", invSrc);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    bol =false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::namedWindow("view2");
  cv::namedWindow("view02");
  cv::namedWindow("view22");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/depth/image_raw", 1, imageCallback);

  image_transport::Subscriber sub2 = it.subscribe("/output", 1, imageCallback2);
  if (!bol)
  {
      bol =true;
      //imgOld =img;
      //split image in layer (channels

     /*code */
  }
  ros::spin();

  //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  //cv::waitKey(30);
  cv::destroyWindow("view");
  cv::destroyWindow("view2");
  cv::destroyWindow("view02");
  cv::destroyWindow("view22");
}
