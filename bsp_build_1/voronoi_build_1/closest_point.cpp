
#include "closest_point.h"
#include <list>

using namespace std;

PointCloudHalfSpace2D::Arc::
    Arc(Vec2f const&o,
        Vec2f const&pt_a,
        Vec2f const&pt_b)
  : o(o),
    r((o-pt_a).Length()),
    pt_a(pt_a),
    pt_b(pt_b) {
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
    // NOTE: We don't actually bounds check, so really this is circle vs line
    const float dist_o = (o-pt).Length();
    if(::fabs(dist_o - r) < 0.0001f)
        return 0;
    return (dist_o < r) ? -1 : 1;
}

namespace {    
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
  : div_o(div_o), div_d(div_d), sorter(div_o, div_d) {
    std::vector<Vec2f> points_sorted_v(points_unsorted);
    std::sort(points_sorted_v.begin(), points_sorted_v.end(), sorter);
    std::list<Vec2f> points_sorted;
    std::copy(points_sorted_v.begin(), points_sorted_v.end(), std::back_inserter(points_sorted));
      for(bool use_front = true;!points_sorted.empty();use_front = !use_front) {
//      for(bool use_front = true;points_sorted.size() > 1;use_front = !use_front) {
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
    fprintf(stderr, "adding %f %f\n", pt.x, pt.y);
    if (points_added_.size() == 0) {
        points_added_.insert(pt);
    } else if(points_added_.size() == 1) {
        AddArcForPoints(*points_added_.begin(), pt);
        points_added_.insert(pt);
    } else if(points_added_.size() > 1) {
        const Arc to_remove =
            (arcs_by_start_pt_.size() > 1) ? arcs_by_start_pt_.find(last_pt_added)->second : arcs_by_start_pt_.begin()->second;
        // TODO: Handle points on same arc efficiently
        bool ruled_out = (to_remove.IsBetweenArcAndLine(pt) > 0);
        fprintf(stderr, "ruled_out %i\n", (int)ruled_out);
        if(!ruled_out) {
            arcs_by_start_pt_.erase(to_remove.pt_a);
            AddArcForPoints(to_remove.pt_a, pt);
            AddArcForPoints(pt, to_remove.pt_b);
            points_added_.insert(pt);
            last_pt_added = pt;
        }
    }
}

void PointCloudHalfSpace2D::AddArcForPoints(Vec2f const&less_pt, Vec2f const&more_pt) {
    assert(sorter(less_pt, more_pt));
    const Vec2f mid = (more_pt + less_pt) / 2.0f;
    const Vec2f dir = (less_pt - more_pt).Normalized();
    const Vec2f perp_dir(-dir.y, dir.x);
    Vec2f int_pt;
    bool intersection = line_intersection(div_o, div_o + div_d, mid, mid + perp_dir, int_pt);
    if(intersection) {
        arcs_by_start_pt_.insert(map<Vec2f, Arc>::value_type(less_pt, Arc(int_pt, less_pt, more_pt)));
    }
}

void PointCloudHalfSpace2D::GetArcs(std::vector<Arc> &output)const {
    for(auto const&it : arcs_by_start_pt_)
        output.push_back(it.second);
}