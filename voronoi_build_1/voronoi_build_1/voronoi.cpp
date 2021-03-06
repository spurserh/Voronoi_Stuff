
#include "voronoi.h"
#include <cassert>
#include <cfloat>

Voronoi::Edge::Edge(Vec2f const&a, Vec2f const&b)
 : pt_a(a), pt_b(b) {
    
}

Voronoi::Edge::Edge(Vec2f const&a, Vec2f const&b, Extrema1f const&extents)
 : pt_a(a), pt_b(b), extents(extents) {
    
}

Voronoi::Voronoi()
  : extents_(Vec2f(FLT_MAX, FLT_MAX), Vec2f(-FLT_MAX, -FLT_MAX))
{
    
}

void Voronoi::Add(Vec2f const&pt) {
    if (points_.find(pt) != points_.end())
        return;
    
    // TODO: This is O(N^3)
    
    extents_ = Extrema2f(Vec2f(FLT_MAX, FLT_MAX), Vec2f(-FLT_MAX, -FLT_MAX));
    
    points_.insert(pt);
    
    for(auto const&pt : points_)
        extents_.DoEnclose(pt);
    
    edges_.clear();
    for(auto const&pt_a : points_) {
        for(auto const&pt_b : points_) {
            // Don't check neighbors twice
            if (&pt_a >= &pt_b)
                continue;
            
            Vec2f mid_pt = (pt_a + pt_b) / 2.0f;
            Vec2f closest_to_mid = BruteClosest(mid_pt);
            if(closest_to_mid == pt_a || closest_to_mid == pt_b) {
                neighbors_.insert(Neighbors::value_type(pt_a, pt_b));
            }
            
            edges_.insert(Edges::value_type(MakeNeighborId(pt_a, pt_b),
                                            MakeEdgeExtents(-FLT_MAX, FLT_MAX)));
        }
    }
    
    // Discover extents
    std::vector<NeighborId> dead_edges;
    for(auto &edge_find : edges_) {
        Extrema1f &extents_to_find = edge_find.second;
        NeighborId const&neighbors_find = edge_find.first;
        // TODO: This is kind of dumb
        const Edge temp_edge_to_find(std::get<0>(neighbors_find), std::get<1>(neighbors_find));
        
        int n_intersect = 0;
        
        for(auto const&edge_b : edges_) {
            // Don't check against self
            if (edge_find.first == edge_b.first)
                continue;
            
            // TODO: This is kind of dumb
            NeighborId const&neighbors_b = edge_b.first;
            const Edge temp_edge_b(std::get<0>(neighbors_b), std::get<1>(neighbors_b));
            
            float t_a;
            if(EdgesIntersect(temp_edge_to_find,
                              temp_edge_b,
                              t_a)) {
                // !! TODO: In multi-line intersections, this is not reliable
                // Try epsilon back/forth
                if(BruteIsBetweenNeighbors(temp_edge_to_find.mid() + temp_edge_to_find.dir() * (t_a + 0.0001f),
                                           neighbors_b) ||
                   BruteIsBetweenNeighbors(temp_edge_to_find.mid() + temp_edge_to_find.dir() * (t_a - 0.0001f),
                                           neighbors_b))
                {
                    static const float find_epsilon = 0.001f;
                    Vec2f min_pt = temp_edge_to_find.mid() + temp_edge_to_find.dir() * (t_a + find_epsilon);
                    // Must check slightly up for min, slightly down for max
                    if (BruteIsBetweenNeighbors(min_pt,
                                                neighbors_find)) {
                        extents_to_find.mMin[0] = std::max(extents_to_find.mMin[0], t_a);
                        extents_.DoEnclose(min_pt);
                        ++n_intersect;
                    }
                    Vec2f max_pt = temp_edge_to_find.mid() + temp_edge_to_find.dir() * (t_a - find_epsilon);
                    if (BruteIsBetweenNeighbors(max_pt,
                                                neighbors_find)) {
                        extents_to_find.mMax[0] = std::min(extents_to_find.mMax[0], t_a);
                        extents_.DoEnclose(max_pt);
                        ++n_intersect;
                    }
                }
            }
        }
        
        if (n_intersect == 0)
            dead_edges.push_back(neighbors_find);
    }
    
    for(NeighborId const&dead_id : dead_edges)
        edges_.erase(dead_id);
}

bool Voronoi::BruteIsBetweenNeighbors(Vec2f const&test_pt, NeighborId const&neighbors)const {
    Vec2f closest = BruteClosest(test_pt);
    return closest == std::get<0>(neighbors) || closest == std::get<1>(neighbors);
}

bool line_intersection(Vec2f p1, Vec2f p2, Vec2f p3, Vec2f p4, Vec2f &out_pt) {
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

inline float Dot(const Vec2f& a,const Vec2f& b)                        { return (a.x*b.x) + (a.y*b.y); }
inline float PerpDot(const Vec2f& a,const Vec2f& b)                    { return (a.y*b.x) - (a.x*b.y); }

bool LineCollision( const Vec2f& A1, const Vec2f& A2,
                   const Vec2f& B1, const Vec2f& B2,
                   float* out                           )
{
    Vec2f a(A2-A1);
    Vec2f b(B2-B1);
    
    float f = PerpDot(a,b);
    if(::fabs(f) < 0.0001f)      // lines are parallel
        return false;
    
    Vec2f c(B2-A2);
    float aa = PerpDot(a,c);
    
    *out = 1.0f - (aa / f);
    return true;
}



bool Voronoi::EdgesIntersect(Edge const&a,
                             Edge const&b,
                             float &t_a) {
    Vec2f i_pt;
    if (!line_intersection(a.mid(), a.mid()+a.dir(),
                           b.mid(), b.mid()+b.dir(),
                           i_pt))
        return false;
    
    t_a = (i_pt - a.mid()).Dot(a.dir());
    return true;
}

void Voronoi::Remove(Vec2f const&pt) {
    assert(!"TODO");
}

Vec2f Voronoi::Closest(Vec2f const&pt) {
    assert(!"TODO");
    return Vec2f(0,0);
}

void Voronoi::GetEdges(std::vector<Voronoi::Edge> &output) {
    for(auto const&edge : edges_) {
        NeighborId const&neighborId = edge.first;
        output.push_back(Edge(std::get<0>(neighborId), std::get<1>(neighborId), edge.second));
    }
}
void Voronoi::GetPoints(std::vector<Vec2f> &output) {
    std::copy(points_.begin(), points_.end(), std::back_inserter(output));
}

Extrema2f Voronoi::GetDiagramDetailExtents()const {
    return extents_;
}

Vec2f Voronoi::BruteClosest(Vec2f const&pt)const {
    Vec2f ret(std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN());
    float dist = FLT_MAX;
    for (Vec2f const&pt_ref : points_) {
        float this_dist = (pt_ref - pt).Length();
        if (this_dist < dist) {
            dist = this_dist;
            ret = pt_ref;
        }
    }
    return ret;
}