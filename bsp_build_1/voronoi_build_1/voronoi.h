//
//  voronoi.h
//  voronoi_build_1
//
//  Created by Sean R Purser-Haskell on 2/1/15.
//  Copyright (c) 2015 Sean R Purser-Haskell. All rights reserved.
//

#ifndef voronoi_build_1_voronoi_h
#define voronoi_build_1_voronoi_h

#include "Vec2f.h"

#include <cfloat>
#include <vector>
#include <map>
#include <set>


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


// TODO: Shared structure / persistence, so 2nd, 3rd, etc, closest can be found
class Voronoi {
public:
    Voronoi();
    
    // Will not add duplicate points
    void Add(Vec2f const&pt);
    void Remove(Vec2f const&pt);

    Vec2f Closest(Vec2f const&pt)const;

    struct Edge {
        Edge(Vec2f const&a, Vec2f const&b);
        Edge(Vec2f const&a, Vec2f const&b, Extrema1f const&extents);
        
        Vec2f pt_a, pt_b;
        // min may be -FLT_MAX, max may be FLT_MAX, if the edge is a ray or a line
        Extrema1f extents;
        
        inline Vec2f closest_pt_on_edge(Vec2f const&pt) const {
            const float closest_t_on_line = (pt - mid()).Dot(dir());
            const float closest_t_on_edge = std::max(extents.mMin[0],
                                              std::min(extents.mMax[0], closest_t_on_line));
            return mid() + dir() * closest_t_on_edge;
        }
        
        inline bool intersects_line(Vec2f const&o, Vec2f const&d) const {
            Vec2f ipt;
            if(!line_intersection(mid(), mid() + dir(), o, o + d, ipt))
                return false;
            const float int_t = (ipt - mid()).Dot(dir());
            return (int_t >= extents.mMin[0]) && (int_t <= extents.mMax[0]);
        }
        
        inline float distance_to_point(Vec2f const&pt) const {
            const Vec2f closest_pt = closest_pt_on_edge(pt);
            return (closest_pt - pt).Length();
        }

        inline Vec2f mid() const {
            return (pt_a + pt_b) / 2.0f;
        }
        
        inline Vec2f dir() const {
            Vec2f a_to_b = (pt_a - pt_b).Normalized();
            return Vec2f(-a_to_b.y, a_to_b.x);
        }
        
        inline Vec2f min_pt(const float max_dim) const {
            const float t = (extents.mMin[0] != -FLT_MAX) ? extents.mMin[0] : -max_dim;
            return mid() + dir() * t;
        }

        inline Vec2f max_pt(const float max_dim) const {
            const float t = (extents.mMax[0] != FLT_MAX) ? extents.mMax[0] : max_dim;
            return mid() + dir() * t;
        }
    };
    
    bool NeighboringPoints(Vec2f const&pt, std::vector<Vec2f> &output)const;
    bool NeighboringEdges(Vec2f const&pt, std::vector<Edge> &output)const;
    
    // anywhere is a point in space which does not necessarily have to have been added via Add()
    // Returns a list of the edges which would be affected if a point were added here
    void EdgesAffectedByAdd(Vec2f const&anywhere,
                            std::vector<Edge> &edges)const;
    // Remove?

    void GetEdges(std::vector<Edge> &output)const;
    void GetPoints(std::vector<Vec2f> &output)const;
    
    // The diagram is actually infinite, but this gets the extents of graph nodes (vertices)
    // If no vertices exist, it will at least be the bounding box of the points provided.
    Extrema2f GetDiagramDetailExtents()const;
    
    // Temp
    static bool EdgesIntersect(Edge const&a,
                               Edge const&b,
                               float &t_a);
    // O(n) closest pt
    Vec2f BruteClosest(Vec2f const&pt)const;

private:
    inline static bool pt_less(Vec2f const&a, Vec2f const&b) {
        if (a.y != b.y)
            return a.y < b.y;
        return a.x < b.x;
    }

    
    typedef std::multimap<Vec2f, Vec2f> Neighbors;
    // Lesser ID must be first, according to pt_less()
    typedef std::tuple<Vec2f, Vec2f> NeighborId;
    typedef std::map<NeighborId, Extrema1f> Edges;
    inline static NeighborId MakeNeighborId(Vec2f const&a, Vec2f const&b) {
        return pt_less(a, b) ? NeighborId(a,b) : NeighborId(b,a);
    }
    inline static Extrema1f MakeEdgeExtents(float min_t, float max_t) {
        return Extrema1f(Vec1f(min_t), Vec1f(max_t));
    }
    bool BruteIsBetweenNeighbors(Vec2f const&test_pt, NeighborId const&neighbors)const;

    // Recursive search
    void EdgesAffectedByAddInternal(const Vec2f const&new_pt,
                                    const Vec2f const&existing_pt,
                                    const float max_dim,
                                    std::set<NeighborId> &edges,
                                    std::set<Vec2f> &points_visited)const;

    
    std::set<Vec2f> points_;
    Neighbors neighbors_;
    Edges edges_;
    Extrema2f extents_;
    
};

#endif
