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

using namespace std;

Voronoi voronoi;
int random_seed = 234;

static void
Init(void)
{
    voronoi = Voronoi();
    /*
    voronoi.Add(Vec2f(-0.6f, -0.5f));
    voronoi.Add(Vec2f(0.3f, -0.3f));
    voronoi.Add(Vec2f(-0.2f, 0.5f));
    voronoi.Add(Vec2f(0,0));
    voronoi.Add(Vec2f(0.65,-0.65));
    voronoi.Add(Vec2f(-0.1, -0.1));

    voronoi.Add(Vec2f(-1, -1));
    voronoi.Add(Vec2f( 1, -1));
    voronoi.Add(Vec2f( 1,  1));
    voronoi.Add(Vec2f( 1, -1));
     */
    /*
     // Random
    srand(random_seed);
    for(int pts = 0;pts<15;++pts)
        voronoi.Add(Vec2f(float(rand()) / float(RAND_MAX), float(rand()) / float(RAND_MAX)));
    */
    /*
    // Square
    for(int row=0;row<5;++row) {
        for(int col=0;col<5;++col) {
            voronoi.Add(Vec2f(col, row) / Vec2f(4,4));
        }
    }
    */
/*
    // Circle
    // Hmm.. O(n) edges affected adding a point over and over..
    for(int r = 0;r<25;++r) {
        const float t = float(r) / float(25);
        const float rads = t * M_PI * 2.0f;
        voronoi.Add(Vec2f(::cos(rads) * 0.75f, ::sin(rads) * 0.75f));
    }
    voronoi.Add(Vec2f(0,0));
*/
    // Spiral
    for(int r = 0;r<25;++r) {
        const float t = float(r) / float(25);
        const float rads = t * M_PI * 2.0f;
        voronoi.Add(Vec2f(::cos(rads) * 0.75f, ::sin(rads) * 0.75f) * (t+0.1));
    }
}

bool view_mode = false;
Vec2f selected_pt(0.4, 0.6);

Extrema2f GetViewingExtents() {
    const Extrema2f extents = voronoi.GetDiagramDetailExtents();
    const Vec2f center = (extents.mMin + extents.mMax) / 2.0f;
    // Make it square
    const float max_dim = std::max(extents.GetSize().x, extents.GetSize().y);
    const float aspect = 800.0f / 600.0f;
    Extrema2f extents_expanded;
    
    if(view_mode) {
        extents_expanded = Extrema2f(center - Vec2f(max_dim, max_dim) * aspect * 0.75f,
                                     center + Vec2f(max_dim, max_dim) * 0.75f);
    } else {
        extents_expanded = Extrema2f(Vec2f(-1.5f * aspect,-1.5f),
                                     Vec2f( 1.5f * aspect, 1.5f));
    }
    return extents_expanded;
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
            voronoi.Add(pt_here);
            glutPostRedisplay();
            break;
    }
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

static void
Draw(void)
{
    glClearColor(1,1,1, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//    gluOrtho2D(-1, 1, 1, -1);
//    gluOrtho2D(-1.5, 1.5, 1.5, -1.5);
//    gluOrtho2D(-5, 5, 5, -5);
    
    const Extrema2f extents_expanded = GetViewingExtents();
    
    gluOrtho2D(extents_expanded.mMin.x,
               extents_expanded.mMax.x,
               extents_expanded.mMax.y,
               extents_expanded.mMin.y);
    
    glMatrixMode(GL_MODELVIEW);
    
    
    // Brute force reference
    static const int nBruteRows = 800;
    static const int nBruteCols = 600;
    glPointSize(1.5f);
    glBegin(GL_POINTS);
    for(int row = 0;row<nBruteRows;++row) {
        for(int col= 0;col<nBruteCols;++col) {
            const Vec2f loc_r(float(col) / float(nBruteCols-1), float(row) / float(nBruteRows-1));
            const Vec2f loc = extents_expanded.mMin + loc_r * extents_expanded.GetSize();
            Vec2f closest = voronoi.BruteClosest(loc);
            SetColorForPt(closest, 0);
            glVertex2fv((float const*)&loc);
        }
    }
    glEnd();

    vector<Vec2f> points;
    voronoi.GetPoints(points);

    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for(Vec2f const&pt : points) {
        glVertex3f(0,0,0);
        glVertex2fv((float const*)&pt);
    }
    glEnd();
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for(Vec2f const&pt : points) {
        SetColorForPt(pt, 0);
        glVertex2fv((float const*)&pt);
    }
    glEnd();
    
    vector<Voronoi::Edge> edges;
    voronoi.GetEdges(edges);
    const float max_dim = std::max(extents_expanded.GetSize().x, extents_expanded.GetSize().y);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for(auto const&edge : edges) {
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
    
    vector<Vec2f> neighbor_pts;
    vector<Voronoi::Edge> neighbor_edges;
    
    const Vec2f closest_selected = voronoi.Closest(selected_pt);
    
    voronoi.NeighboringPoints(closest_selected, neighbor_pts);
    voronoi.NeighboringEdges(closest_selected, neighbor_edges);
    
    glPointSize(6.0f);
    glBegin(GL_POINTS);
    glColor3f(1,1,1);
    glVertex2fv((float const*)&closest_selected);
    glColor3f(1,0,0);
    for(Vec2f const&pt : neighbor_pts)
        glVertex2fv((float const*)&pt);
    glEnd();
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1,0.2,0.2);
    for(auto const&edge : neighbor_edges) {
        Vec2f min_pt, max_pt;
        GetSaneEdgeVerts(edge, max_dim, min_pt, max_pt);
        glVertex2fv((float const*)(&min_pt));
        glVertex2fv((float const*)(&max_pt));
    }
    glEnd();
    
    {
        vector<Voronoi::Edge> affected_edges;
        voronoi.EdgesAffectedByAdd(selected_pt, affected_edges);
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        glColor3f(1,1,1);
        for(auto const&edge : affected_edges) {
            Vec2f min_pt, max_pt;
            GetSaneEdgeVerts(edge, max_dim, min_pt, max_pt);
            glVertex2fv((float const*)(&min_pt));
            glVertex2fv((float const*)(&max_pt));
        }
        glEnd();
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