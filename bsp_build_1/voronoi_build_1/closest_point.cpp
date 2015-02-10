
#include "closest_point.h"
#include <list>

PointCloudHalfSpace2D::Arc::
    Arc(Vec2f const&o,
        Vec2f const&pt_a,
        Vec2f const&pt_b)
  : o(o),
    r((o-pt_a).Length()){
    const float rads_a = ::atan2(pt_a.y - o.y, pt_a.x - o.x);
    const float rads_b = ::atan2(pt_b.y - o.y, pt_b.x - o.x);
    a_min_rads = std::min(rads_a, rads_b);
    a_max_rads = std::max(rads_a, rads_b);
}


Vec2f PointCloudHalfSpace2D::Arc::Point(float t)const {
    const float a = a_min_rads + t * (a_max_rads - a_min_rads);
    return o + Vec2f(::cos(a), ::sin(a)) * r;
}

// < 0 is between line and arc,
// = 0 is exactly on the arc.
// > 0 is beyond the arc, away from the line.
int PointCloudHalfSpace2D::Arc::IsBetweenArcAndLine(Vec2f const&pt)const {
    return 0;
}

namespace {
    struct sort_by_pos_dir {
        sort_by_pos_dir(Vec2f const&o, Vec2f const&d)
          : o(o), d(d) {
        }
        bool operator() (Vec2f const&a, Vec2f const&b) {
            const float t_a = (a-o).Dot(d);
            const float t_b = (b-o).Dot(d);
            return t_a < t_b;
        }
        const Vec2f o, d;
    };
    
    inline bool line_intersection(Vec2f p1, Vec2f p2, Vec2f p3, Vec2f p4, Vec2f &out_pt) {
        // Store the values for fast access and easy
        // equations-to-code conversion
        float x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
        float y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;
        
        float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        // If d is zero, there is no intersection
        if (::fabs(d) < 0.0001f) return false;
        
        // Get the x and y
        float pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
        float x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
        float y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;
        
        out_pt.x = x;
        out_pt.y = y;
        return true;
    }

}

PointCloudHalfSpace2D::PointCloudHalfSpace2D(Vec2f const&div_o,
                                             Vec2f const&div_d,
                                             std::vector<Vec2f> const&points_unsorted)
  : div_o(div_o), div_d(div_d) {
    std::vector<Vec2f> points_sorted_v(points_unsorted);
    std::sort(points_sorted_v.begin(), points_sorted_v.end(), sort_by_pos_dir(div_o, div_d));
    std::list<Vec2f> points_sorted;
    std::copy(points_sorted_v.begin(), points_sorted_v.end(), std::back_inserter(points_sorted));
      for(bool use_front = true;!points_sorted.empty();use_front = !use_front) {
        if(use_front) {
          AddPointIfNotRuledOut(points_sorted.front());
          points_sorted.pop_front();
        } else {
          AddPointIfNotRuledOut(points_sorted.back());
          points_sorted.pop_back();
        }
    }
}

void PointCloudHalfSpace2D::AddPointIfNotRuledOut(Vec2f const&pt) {
    // TODO: Need to check if ruled out
    // TODO: Need to remove arcs

    if (points_added_.size() == 0) {
        points_added_.push_back(pt);
    } else if(points_added_.size() == 1) {
        Vec2f const&o_pt = points_added_[0];
        const Vec2f mid = (pt + o_pt) / 2.0f;
        const Vec2f dir = (o_pt - pt).Normalized();
        const Vec2f perp_dir(-dir.y, dir.x);
        Vec2f int_pt;
        // TODO: Check if ruled out
        bool intersection = line_intersection(div_o, div_o + div_d, mid, mid + perp_dir, int_pt);
        if(intersection) {
            arcs_.push_back(Arc(int_pt, pt, o_pt));
            points_added_.push_back(pt);
        }
    } else {
        // TODO
    }
}

void PointCloudHalfSpace2D::GetArcs(std::vector<Arc> &output)const {
    std::copy(arcs_.begin(), arcs_.end(), std::back_inserter(output));
}