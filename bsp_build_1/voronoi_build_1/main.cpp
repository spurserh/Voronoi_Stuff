//
//  main.cpp
//  voronoi_build_1
//
//  Created by Sean R Purser-Haskell on 2/1/15.
//  Copyright (c) 2015 Sean R Purser-Haskell. All rights reserved.
//

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <GLUT/glut.h>
#include <cassert>
#include <cstdio>
#include <vector>

#include "Vec2f.h"
#include "voronoi.h"

#include "closest_point.h"

using namespace std;

int random_seed = 234;
Vec2f div_o(0,0.5), div_d(1,0);
vector<Vec2f> points;


namespace {
bool BruteIsBorder(Vec2f const&loc,
                   Vec2f const&div_o,
                   Vec2f const&div_d,
                   Voronoi const&voronoi) {
    Voronoi temp_v = voronoi;
    temp_v.Add(loc);
    vector<Voronoi::Edge> neighbor_edges;
    temp_v.NeighboringEdges(loc, neighbor_edges);
    for (Voronoi::Edge const&edge : neighbor_edges) {
        if(edge.intersects_line(div_o, div_d))
            return true;
    }
    return false;
}

bool OnPositiveSide(Vec2f const&loc,
                    Vec2f const&div_o,
                    Vec2f const&div_d) {
    return (loc - div_o).Dot(Vec2f(-div_d.y, div_d.x)) >= 0.0f;
}
}

bool view_mode = false;
Vec2f selected_pt(0.4, 0.6);
Vec2i resolution(200 * 0.5, 150 * 0.5);
//Vec2i resolution(200  * 1.5, 150 * 1.5);

// -0.306667 -0.380000 b 0.413333 0.360000 c 0.223333 -0.083333
/*
Vec2f int_test_a(-0.306667, -0.380000),
      int_test_b(0.413333, 0.360000),
      int_test_c(0.223333, -0.083333);
*/

Vec2f int_test_a(0,0),
        int_test_b(0,0),
        int_test_c(0,0);

static void
Init(void)
{
    points.clear();
    
    points.push_back(Vec2f(-0.960000, -0.293333));
    points.push_back(Vec2f(-0.423333, -0.666667));
    points.push_back(Vec2f(0.610000, 0.060000));
    points.push_back(Vec2f(1.880000, -0.350000));
}

Extrema2f GetViewingExtents() {
    const float aspect = float(resolution.width) / float(resolution.height);
    return Extrema2f(Vec2f(-1.0f * aspect, -1),
                     Vec2f( 1.0f * aspect,  1));
}

/* ARGSUSED1 */
static void
Key(unsigned char key, int x, int y)
{
    const Extrema2f extents = GetViewingExtents();
    const Vec2f pt_here = extents.mMin +
                            extents.GetSize() * (Vec2f(float(x), float(y)) / Vec2f(float(800), float(600)));
    switch (key) {
        case 27:
            exit(0);
        case ' ':
            Init();
            points.clear();
            fprintf(stderr, "---\n");
            glutPostRedisplay();
            break;
        case 'n':
            ++random_seed;
            Init();
            glutPostRedisplay();
            break;
        case 's':
            selected_pt = pt_here;
            glutPostRedisplay();
            break;
        case 'a':
            points.push_back(pt_here);
            fprintf(stderr, "add %f %f\n", pt_here.x, pt_here.y);
            glutPostRedisplay();
            break;
        case '1':
            int_test_a = pt_here;
            glutPostRedisplay();
            break;
        case '2':
            int_test_b = pt_here;
            glutPostRedisplay();
            break;
        case '3':
            int_test_c = pt_here;
            glutPostRedisplay();
            break;
    }
    fprintf(stderr, "int a %f %f b %f %f c %f %f\n",
            int_test_a.x, int_test_a.y,
            int_test_b.x, int_test_b.y,
            int_test_c.x, int_test_c.y);
}

void SetColorForPt(Vec2f const&pt, float b) {
    glColor3f(float(uint64_t(pt.x * float(10000)) % uint64_t(352)) / float(352),
              float(uint64_t(pt.y * float(10000)) % uint64_t(142)) / float(142),
              b);
}

void GetSaneEdgeVerts(Voronoi::Edge const&edge,
                      float const&max_dim,
                      Vec2f &min_pt,
                      Vec2f &max_pt) {
    Extrema1f sane_extents(Vec1f(std::max(-max_dim*2, edge.extents.mMin[0])),
                           Vec1f(std::min( max_dim*2, edge.extents.mMax[0])));
    min_pt = edge.mid() + edge.dir() * sane_extents.mMin[0];
    max_pt = edge.mid() + edge.dir() * sane_extents.mMax[0];
}


Vec2f closest_pt_on_line(Vec2f const&pt, Vec2f const&line_o, Vec2f const&line_d) {
    const float closest_t = (pt - line_o).Dot(line_d);
    return line_o + line_d * closest_t;
}

Vec2f BruteClosest(Vec2f const&location, std::vector<Vec2f> const&points) {
    Vec2f ret;
    float min_d = FLT_MAX;
    for(Vec2f const&p : points) {
        const float d = (location - p).Length();
        if(d < min_d) {
            min_d = d;
            ret = p;
        }
    }
    return ret;
}

// border points experiment
static void
Draw(void)
{
    glClearColor(1,1,1, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //    gluOrtho2D(0, 1, 1, 0);
    //    gluOrtho2D(-1.5, 1.5, 1.5, -1.5);
    //    gluOrtho2D(-5, 5, 5, -5);
    
    const Extrema2f extents_expanded = GetViewingExtents();
    
    gluOrtho2D(extents_expanded.mMin.x,
               extents_expanded.mMax.x,
               extents_expanded.mMax.y,
               extents_expanded.mMin.y);
    
    glMatrixMode(GL_MODELVIEW);
    
    Voronoi pos_voronoi, neg_voronoi;
    
    for(Vec2f const&pt : points) {
        if(OnPositiveSide(pt, div_o, div_d)) {
            pos_voronoi.Add(pt);
        } else {
            neg_voronoi.Add(pt);
        }
    }

    vector<Voronoi::Edge> pos_edges, neg_edges;
    pos_voronoi.GetEdges(pos_edges);
    neg_voronoi.GetEdges(neg_edges);

    glPointSize(2);
    glBegin(GL_POINTS);
    for(int row=0;row<resolution.height;++row) {
        for(int col=0;col<resolution.width;++col) {
            // Border point if it can be the closest point to anywhere on or beyond the line.
            const Vec2f loc_r(float(col) / float(resolution.width-1), float(row) / float(resolution.height-1));
            const Vec2f loc = extents_expanded.mMin + loc_r * extents_expanded.GetSize();
            
            bool side = OnPositiveSide(loc, div_o, div_d);
            Voronoi const&v = side ? pos_voronoi : neg_voronoi;
            bool is_border = BruteIsBorder(loc, div_o, div_d, v);
            
//            glColor3f(0, is_border ? 0.5f : 0.75f, side ? 0.5f : 0.75f);
            glColor3f(is_border ? 0 : 1, 0,0);
            glVertex2f(loc.x, loc.y);
        }
    }
    glEnd();
    
    glColor3f(0,0,0);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    Vec2f line_a = div_o + div_d * 10;
    Vec2f line_b = div_o - div_d * 10;
    glVertex2fv((float const*)&line_a);
    glVertex2fv((float const*)&line_b);
    glEnd();
    
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for(Vec2f const&pt : points) {
        SetColorForPt(pt, 0);
        glVertex2fv((float const*)&pt);
    }
    glEnd();
    
    
    const float max_dim = std::max(extents_expanded.GetSize().x, extents_expanded.GetSize().y);
    glLineWidth(1.0f);
    glColor3f(0,0,0);
    glBegin(GL_LINES);
    for(auto const&edge : pos_edges) {
        Extrema1f sane_extents(Vec1f(std::max(-max_dim*2, edge.extents.mMin[0])),
                               Vec1f(std::min( max_dim*2, edge.extents.mMax[0])));
        Vec2f min_pt = edge.mid() + edge.dir() * sane_extents.mMin[0];
        Vec2f max_pt = edge.mid() + edge.dir() * sane_extents.mMax[0];
        GetSaneEdgeVerts(edge, max_dim, min_pt, max_pt);
        glColor4f(1,0,0,1);
        glVertex2fv((float const*)(&min_pt));
        glColor4f(0,1,0,1);
        glVertex2fv((float const*)(&max_pt));
    }
    for(auto const&edge : neg_edges) {
        Extrema1f sane_extents(Vec1f(std::max(-max_dim*2, edge.extents.mMin[0])),
                               Vec1f(std::min( max_dim*2, edge.extents.mMax[0])));
        Vec2f min_pt = edge.mid() + edge.dir() * sane_extents.mMin[0];
        Vec2f max_pt = edge.mid() + edge.dir() * sane_extents.mMax[0];
        GetSaneEdgeVerts(edge, max_dim, min_pt, max_pt);
        glColor4f(1,0,0,1);
        glVertex2fv((float const*)(&min_pt));
        glColor4f(0,1,0,1);
        glVertex2fv((float const*)(&max_pt));
    }
    glEnd();
    
    {
        glLineWidth(2.0f);
        const float half_dist = (int_test_a - int_test_b).Length() / 2.0f;
        const Vec2f midpt = (int_test_a + int_test_b) / 2.0f;
        const Vec2f dir = (int_test_b - int_test_a).Normalized();
        const Vec2f perp_dir = Vec2f(-dir.y, dir.x) * 0.1f;
        const Voronoi::Edge test_edge(midpt - perp_dir,
                                      midpt + perp_dir,
                                      Extrema1f(Vec1f(-half_dist), Vec1f(half_dist)));
        const bool intersects = test_edge.intersects_line(div_o, div_d);
        glColor3f(0, 1, intersects ? 1 : 0);
        glBegin(GL_LINES);
        glVertex2f(int_test_a.x, int_test_a.y);
        glVertex2f(int_test_b.x, int_test_b.y);
        glEnd();
        glColor3f(1, 0, 0);
        glBegin(GL_LINES);
        glVertex2f(test_edge.pt_a.x, test_edge.pt_a.y);
        glVertex2f(test_edge.pt_b.x, test_edge.pt_b.y);
        glEnd();
    }
    
    {
        glColor3f(0, 0, 1);
        glBegin(GL_LINE_STRIP);
        Vec2f dir = (int_test_b - int_test_a).Normalized();
        Vec2f perp_dir(-dir.y, dir.x);
        PointCloudHalfSpace2D::Arc test_par(int_test_a, int_test_b, int_test_c);
        for(int col=0;col<resolution.width;++col) {
            const Vec2f loc = test_par.Point(col / float (resolution.width - 1));
            glVertex2fv((const float*)&loc);
        }
        glEnd();
    }
    
    {
        PointCloudHalfSpace2D halfspace(div_o, div_d, points);
        std::vector<PointCloudHalfSpace2D::Arc> arcs;
        halfspace.GetArcs(arcs);
        for(PointCloudHalfSpace2D::Arc const&test_par : arcs)
        {
            glColor3f(0, 0, 1);
            glBegin(GL_LINE_STRIP);
            for(int col=0;col<resolution.width;++col) {
                const Vec2f loc = test_par.Point(col / float (resolution.width - 1));
                glVertex2fv((const float*)&loc);
            }
            glEnd();
        }
    }
    
    fprintf(stderr, "--- drew\n");
    
    glutSwapBuffers();
}

int
main(int argc, char **argv)
{
    glutInit(&argc, argv);
    
    // hidpi not working..
    glutInitDisplayString("rgba double samples=8 hidpi");
    glutInitWindowSize(800, 600);
    glutCreateWindow("ABGR extension");
    if (!glutExtensionSupported("GL_EXT_abgr")) {
        printf("Couldn't find abgr extension.\n");
        exit(0);
    }
#if !GL_EXT_abgr
    printf("WARNING: client-side OpenGL has no ABGR extension support!\n");
    printf("         Drawing only RGBA (and not ABGR) images and textures.\n");
#endif
    Init();
    glutKeyboardFunc(Key);
    glutDisplayFunc(Draw);
    glutMainLoop();
    return 0;             /* ANSI C requires main to return int. */
}