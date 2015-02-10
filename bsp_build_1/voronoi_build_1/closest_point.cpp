
#include "closest_point.h"

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

PointCloudHalfSpace2D::PointCloudHalfSpace2D(Vec2f const&div_o,
                                             Vec2f const&div_d,
                                             std::vector<Vec2f> const&points) {
    // TODO: The ordering needs to be smart for actually building the data structure
    // TODO
}

void PointCloudHalfSpace2D::GetArcs(std::vector<Arc> &output)const {
    std::copy(arcs_.begin(), arcs_.end(), std::back_inserter(output));
}