#ifndef bsp_build_1_closest_point_h
#define bsp_build_1_closest_point_h

#include "Vec2f.h"
#include <memory>
#include <vector>
#include <cfloat>
#include <map>
#include <set>

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


struct sort_by_pos_dir {
    sort_by_pos_dir(Vec2f const&o, Vec2f const&d)
    : o(o), d(d) {
    }
    bool operator() (Vec2f const&a, Vec2f const&b)const {
        const float t_a = (a-o).Dot(d);
        const float t_b = (b-o).Dot(d);
        return t_a < t_b;
    }
    const Vec2f o, d;
};


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
        
        // Convenience
        const Vec2f pt_a, pt_b;
    };

    PointCloudHalfSpace2D(Vec2f const&div_o,
                          Vec2f const&div_d,
                          std::vector<Vec2f> const&points);

    void GetArcs(std::vector<Arc> &output)const;
private:
    const Vec2f div_o, div_d;
    const sort_by_pos_dir sorter;
    // Temp
    std::set<Vec2f> points_added_;
    std::map<Vec2f, Arc> arcs_by_start_pt_;
    Vec2f last_pt_added;
    
    void AddPointIfNotRuledOut(Vec2f const&pt);
    void AddArcForPoints(Vec2f const&less_pt, Vec2f const&more_pt);
    bool RuledOut(Vec2f const&pt)const;
};


#endif
