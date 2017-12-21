#pragma once

#include <opencv2/opencv.hpp>

class Polynom()
{
    /*
     * The polynom coefficients
     */
    float m_c0;
    float m_c1;
    float m_c2;

public:

    /*
     * Get the tangent vector at the specified
     * point
     */
    cv::Point2f GetTangetAt();

    /*
     * Get the perpendicular vector to the tanget
     * at the specified point
     */
    cv::Point2f GetTangetAt();

    /*
     * Get the perpendicular vector to the tanget
     * at the specified point
     */
    cv::Point2f GetTangetAt();
    

}