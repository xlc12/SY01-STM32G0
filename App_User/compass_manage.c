#include "compass_manage.h"

extern  float Target_Azimuth; //校准后的最终角度

//标定偏移值
extern float Calibration_Offset; //校准偏移量




/*
 * @brief: 获取标定后的指南针角度
 * @param: None
 * @return: uint8_t 指南针角度
 */
float getCompassAngle()
{
    
    
    return Target_Azimuth + Calibration_Offset; //校准后的最终角度 ;
}

//获取磁力计原始角度
float getCompassAngle_Raw()
{
    return Target_Azimuth;
}

/*
 * @brief: 获取指南针方位
 * @param: None
 * @return: uint8_t 指南针方位
 * 0°为正北，顺时针递增，360°为一圈，将磁力角度划分为8快，分别为北：1，东北：2，东：3，东南：4，南：5，西南：6，西：7，西北：8；、
 * 以每块为中心占45°，例如337.5°-22.5°为北，22.5°-67.5°为东北，67.5°-112.5°为东，112.5°-157.5°为东南，157.5°-202.5°为南，202.5°-247.5°为西南，247.5°-292.5°为西，292.5°-337.5°为西北。
 */
uint8_t getCompassDirection ()
{
    float azimuth = getCompassAngle();
    // azimuth=225;
    uint8_t direction = 0;
    if (azimuth >= 337.5 || azimuth < 22.5)
    {
        direction = 1;
    }
    else if (azimuth >= 22.5 && azimuth < 67.5)
    {
        direction = 2;
    }
    else if (azimuth >= 67.5 && azimuth < 112.5)
    {
        direction = 3;
    }
    else if (azimuth >= 112.5 && azimuth < 157.5)
    {
        direction = 4;
    }
    else if (azimuth >= 157.5 && azimuth < 202.5)
    {
        direction = 5;
    }
    else if (azimuth >= 202.5 && azimuth < 247.5)
    {
        direction = 6;
    }
    else if (azimuth >= 247.5 && azimuth < 292.5)
    {
        direction = 7;
    }
    else if (azimuth >= 292.5 && azimuth < 337.5)
    {
        direction = 8;
    }
    return direction;
    



}