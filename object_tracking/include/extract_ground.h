#ifndef extract_ground_H
#define extract_ground_H
#include <object_tracking/Road_extract.h>
class Cell{
private:
    float smoothed;
    float height;  // 高度
    float hDiff;  //高度差
    float hGround;  // 
    float minZ;
    float maxZ;
    float innerDiff;
    bool isGround; // 是不是Ground
    bool isCover;
    bool isNeighbor;


public:
    Cell();
    int size;
    int exce_size;
    bool car;
    void updateMinZ(float z){if (z < minZ) minZ = z;}  // 得到z值点的最小值
    void updateMaxZ(float z){if (z > maxZ) maxZ = z;}
    void updataHeight(float h) {height = h;}
    void updateSmoothed(float s) {smoothed = s;} //
    void updateHDiff(float hd){hDiff = hd;} //  赋值hDiff
    void updateGround(){isGround = true; hGround = height;}  // 是地面点
    void updateNeighbor(){isNeighbor = true;}
    void updateInnerDiff(float diff) { innerDiff = diff;}
    void updateCover(){isCover = true;}

    bool isThisGround(){return isGround;}  //判断Ground
    bool isThisCover(){return isCover;}
    bool isThisNeighbor(){return isNeighbor;}
    float getMinZ() {return minZ;}
    float getMaxZ() {return maxZ;}
    float getHeight(){return height;}   // height
    float getHDiff(){ return hDiff;}
    float getInnerDiff() {return innerDiff;}
    float getSmoothed() {return smoothed;}  // 平滑
    float getHGround() {return hGround;}
    
};












#endif
