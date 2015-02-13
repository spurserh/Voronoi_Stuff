
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

PointCloudHalfSpace2D::Arc::Arc() {}

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

    Vec2f ClosestPointOnLine(Vec2f const&pt, Vec2f const&o, Vec2f const&d) {
        const float t = (pt-o).Dot(d);
        return o+d*t;
    }
}

PointCloudHalfSpace2D::PointCloudHalfSpace2D(Vec2f const&div_o,
                                             Vec2f const&div_d,
                                             std::vector<Vec2f> const&points_unsorted)
  : div_o(div_o), div_d(div_d), sorter(div_o, div_d) {

      fprintf(stderr, "----\n");
      for(Vec2f const&pt : points_unsorted) {
          fprintf(stderr, "\tpoints.push_back(Vec2f(%f,%f));\n", pt.x, pt.y);
      }
      fprintf(stderr, "\n");
      
    if(points_unsorted.size() >= 2) {
        std::vector<Vec2f> points_sorted_v(points_unsorted);
        std::sort(points_sorted_v.begin(), points_sorted_v.end(), sorter);
        std::list<Vec2f> points_sorted;
        std::copy(points_sorted_v.begin(), points_sorted_v.end(), std::back_inserter(points_sorted));
        
        // First arc is a special case
        {
            mod_arc_start_pt_ = points_sorted.front();
            AddArc(ArcForPoints(points_sorted.front(), points_sorted.back()));
            points_sorted.pop_front();
            points_sorted.pop_back();
        }
        
        while(points_sorted.size() > 0) {
            Arc prev_arc = arcs_by_start_pt_[mod_arc_start_pt_];
            if (points_sorted.size() == 1) {
                Vec2f const&pt = points_sorted.front();
                if(prev_arc.IsBetweenArcAndLine(pt) <= 0) {
                    arcs_by_start_pt_.erase(mod_arc_start_pt_);
                    AddArc(ArcForPoints(prev_arc.pt_a, pt));
                    AddArc(ArcForPoints(pt, prev_arc.pt_b));
                }
                // Last one so no need for book-keeping
                break;
            } else {
                // Need to tie-break
                Vec2f front_pt = points_sorted.front();
                Vec2f back_pt = points_sorted.back();
             //   const float front_d = (front_pt - ClosestPointOnLine(front_pt, div_o, div_d)).Length();
               // const float back_d = (back_pt - ClosestPointOnLine(back_pt, div_o, div_d)).Length();
                Arc front_arcs[2] = {
                    ArcForPoints(prev_arc.pt_a, front_pt),
                    ArcForPoints(front_pt, prev_arc.pt_b),
                };
                Arc back_arcs[2] = {
                    ArcForPoints(prev_arc.pt_a, back_pt),
                    ArcForPoints(back_pt, prev_arc.pt_b),
                };
                
                // TODO: Handle vertically oriented points
                // TODO: Handle points on same arc efficiently
                
                // TODO: Consider both arcs?
                bool front_ruled_out = (prev_arc.IsBetweenArcAndLine(front_pt) > 0 ||
                                        back_arcs[0].IsBetweenArcAndLine(front_pt) > 0);
                bool back_ruled_out = (prev_arc.IsBetweenArcAndLine(back_pt) > 0 ||
                                       front_arcs[1].IsBetweenArcAndLine(back_pt) > 0);
                
                if(front_ruled_out && back_ruled_out) {
                    points_sorted.pop_front();
                    points_sorted.pop_back();
                    continue;
                }
                
                if(front_ruled_out) {
                    points_sorted.pop_front();
                    continue;
                } else if(back_ruled_out) {
                    points_sorted.pop_back();
                    continue;
                } else {
                    arcs_by_start_pt_.erase(mod_arc_start_pt_);
                    // Need to tie-break
//                    const bool front_first = front_d < back_d;
                    
                    AddArc(front_arcs[0]);
                    AddArc(ArcForPoints(front_pt, back_pt));
                    mod_arc_start_pt_ = front_pt;
                    AddArc(back_arcs[1]);
                    points_sorted.pop_front();
                    points_sorted.pop_back();
                }
            }
        }
    }
}

PointCloudHalfSpace2D::Arc PointCloudHalfSpace2D::ArcForPoints(Vec2f const&less_pt, Vec2f const&more_pt)const {
    assert(sorter(less_pt, more_pt));
    const Vec2f mid = (more_pt + less_pt) / 2.0f;
    const Vec2f dir = (less_pt - more_pt).Normalized();
    const Vec2f perp_dir(-dir.y, dir.x);
    Vec2f int_pt;
    bool intersection = line_intersection(div_o, div_o + div_d, mid, mid + perp_dir, int_pt);
    assert(intersection);
    return Arc(int_pt, less_pt, more_pt);
}

void PointCloudHalfSpace2D::AddArc(Arc const&arc) {
    arcs_by_start_pt_.insert(map<Vec2f, Arc>::value_type(arc.pt_a, arc));
}

void PointCloudHalfSpace2D::GetArcs(std::vector<Arc> &output)const {
    for(auto const&it : arcs_by_start_pt_)
        output.push_back(it.second);
}