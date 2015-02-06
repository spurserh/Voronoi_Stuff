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

#include <vector>
#include <map>
#include <set>

// TODO: Shared structure / persistence, so 2nd, 3rd, etc, closest can be found
class Voronoi {
public:
    Voronoi();
    
    // Will not add duplicate points
    void Add(Vec2f const&pt);
    void Remove(Vec2f const&pt);

    Vec2f Closest(Vec2f const&pt);

    struct Edge {
        Edge(Vec2f const&a, Vec2f const&b);
        Edge(Vec2f const&a, Vec2f const&b, Extrema1f const&extents);
        
        Vec2f pt_a, pt_b;
        // min may be -FLT_MAX, max may be FLT_MAX, if the edge is a ray or a line
        Extrema1f extents;

        inline Vec2f mid() const {
            return (pt_a + pt_b) / 2.0f;
        }
        
        inline Vec2f dir() const {
            Vec2f a_to_b = (pt_a - pt_b).Normalized();
            return Vec2f(-a_to_b.y, a_to_b.x);
        }
    };
    
    void GetEdges(std::vector<Edge> &output);
    void GetPoints(std::vector<Vec2f> &output);
    
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
    inline bool pt_less(Vec2f const&a, Vec2f const&b) {
        if (a.y != b.y)
            return a.y < b.y;
        return a.x < b.x;
    }

    
    typedef std::multimap<Vec2f, Vec2f> Neighbors;
    // Lesser ID must be first, according to pt_less()
    typedef std::tuple<Vec2f, Vec2f> NeighborId;
    typedef std::map<NeighborId, Extrema1f> Edges;
    inline NeighborId MakeNeighborId(Vec2f const&a, Vec2f const&b) {
        return pt_less(a, b) ? NeighborId(a,b) : NeighborId(b,a);
    }
    inline Extrema1f MakeEdgeExtents(float min_t, float max_t) {
        return Extrema1f(Vec1f(min_t), Vec1f(max_t));
    }
    bool BruteIsBetweenNeighbors(Vec2f const&test_pt, NeighborId const&neighbors)const;

    std::set<Vec2f> points_;
    Neighbors neighbors_;
    Edges edges_;
    Extrema2f extents_;
    
};

#endif
