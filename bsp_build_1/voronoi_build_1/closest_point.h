#ifndef bsp_build_1_closest_point_h
#define bsp_build_1_closest_point_h

#include "Vec2f.h"
#include <memory>
#include <vector>
#include <cfloat>
#include <map>

struct PointClassification {
    inline PointClassification(bool side, bool border)
      : side(side), border(border) {
    }
    
    bool side;
    bool border;
};

void ClassifyPoints(Vec2f const&div_o,
                    Vec2f const&div_d,
                    std::vector<Vec2f> const&points,
                    std::map<Vec2f, PointClassification> &output);

struct PointCloudHalfSpace2D {
    struct Arc {
        Arc(Vec2f const&o,
            Vec2f const&pt_a,
            Vec2f const&pt_b);
        
        // t is from 0 to 1
        Vec2f Point(float t)const;
        
        // < 0 is between line and arc,
        // = 0 is exactly on the arc.
        // > 0 is beyond the arc, away from the line.
        int IsBetweenArcAndLine(Vec2f const&pt)const;
        
        Vec2f o;
        float r;
        float a_min_rads;
        float a_max_rads;
    };

    PointCloudHalfSpace2D(Vec2f const&div_o,
                          Vec2f const&div_d,
                          std::vector<Vec2f> const&points);

    void GetArcs(std::vector<Arc> &output)const;
private:
    const Vec2f div_o, div_d;
    // Temp
    std::vector<Vec2f> points_added_;
    std::vector<Arc> arcs_;
    
    void AddPointIfNotRuledOut(Vec2f const&pt);
};


#endif
