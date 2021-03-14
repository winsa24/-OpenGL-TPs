#ifndef MESH_H
#define MESH_H

#include <glad/glad.h>
#include <vector>
#include <memory>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <map>
#include <set>
#include <cstdio>
#include <iostream>

class Mesh {
public:
    virtual ~Mesh();

    const std::vector<glm::vec3> &vertexPositions() const { return _vertexPositions; }
    std::vector<glm::vec3> &vertexPositions() { return _vertexPositions; }

    const std::vector<glm::vec3> &vertexNormals() const { return _vertexNormals; }
    std::vector<glm::vec3> &vertexNormals() { return _vertexNormals; }

    const std::vector<glm::vec2> &vertexTexCoords() const { return _vertexTexCoords; }
    std::vector<glm::vec2> &vertexTexCoords() { return _vertexTexCoords; }

    const std::vector<glm::uvec3> &triangleIndices() const { return _triangleIndices; }
    std::vector<glm::uvec3> &triangleIndices() { return _triangleIndices; }

    /// Compute the parameters of a sphere which bounds the mesh
    void computeBoundingSphere(glm::vec3 &center, float &radius) const;

    void recomputePerVertexNormals(bool angleBased = false);
    void recomputePerVertexTextureCoordinates( );

    void init();
    void initOldGL();
    void render();
    void clear();

    void addPlan(float square_half_side = 1.0f);






    // -------------------------------------------------------------------------- //
    // -------------------------------------------------------------------------- //
    //          MESH SUBDIVISION PRACTICAL
    // -------------------------------------------------------------------------- //
    // -------------------------------------------------------------------------- //

    void subdivideLinear() {
        std::vector<glm::vec3> newVertices = _vertexPositions;
        std::vector<glm::uvec3> newTriangles;

        struct Edge {
            unsigned int a , b;
            Edge( unsigned int c , unsigned int d ) : a( std::min<unsigned int>(c,d) ) , b( std::max<unsigned int>(c,d) ) {}
            bool operator < ( Edge const & o ) const {   return a < o.a  ||  (a == o.a && b < o.b);  }
            bool operator == ( Edge const & o ) const {   return a == o.a  &&  b == o.b;  }
        };
        std::map< Edge , unsigned int > newVertexOnEdge;
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];


            Edge Eab(a,b);
            unsigned int oddVertexOnEdgeEab = 0;
            if( newVertexOnEdge.find( Eab ) == newVertexOnEdge.end() ) {
                newVertices.push_back( (_vertexPositions[ a ] + _vertexPositions[ b ]) / 2.f );
                oddVertexOnEdgeEab = newVertices.size() - 1;
                newVertexOnEdge[Eab] = oddVertexOnEdgeEab;
            }
            else { oddVertexOnEdgeEab = newVertexOnEdge[Eab]; }


            Edge Ebc(b,c);
            unsigned int oddVertexOnEdgeEbc = 0;
            if( newVertexOnEdge.find( Ebc ) == newVertexOnEdge.end() ) {
                newVertices.push_back( (_vertexPositions[ b ] + _vertexPositions[ c ]) / 2.f );
                oddVertexOnEdgeEbc = newVertices.size() - 1;
                newVertexOnEdge[Ebc] = oddVertexOnEdgeEbc;
            }
            else { oddVertexOnEdgeEbc = newVertexOnEdge[Ebc]; }


            Edge Eca(c,a);
            unsigned int oddVertexOnEdgeEca = 0;
            if( newVertexOnEdge.find( Eca ) == newVertexOnEdge.end() ) {
                newVertices.push_back( (_vertexPositions[ c ] + _vertexPositions[ a ]) / 2.f );
                oddVertexOnEdgeEca = newVertices.size() - 1;
                newVertexOnEdge[Eca] = oddVertexOnEdgeEca;
            }
            else { oddVertexOnEdgeEca = newVertexOnEdge[Eca]; }


            // set new triangles :
            newTriangles.push_back( glm::uvec3( a , oddVertexOnEdgeEab , oddVertexOnEdgeEca ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , b , oddVertexOnEdgeEbc ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEca , oddVertexOnEdgeEbc , c ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , oddVertexOnEdgeEbc , oddVertexOnEdgeEca ) );
        }

        // after that:
        _triangleIndices = newTriangles;
        _vertexPositions = newVertices;
        recomputePerVertexNormals( );
        recomputePerVertexTextureCoordinates( );
    }



    // THIS VERSION IS ACCEPTABLE. IT REQUIRES ITERATING ONLY ONTO THE TRIANGLES BUT DOES NOT HANDLE BOUNDARIES.
    void subdivideLoop_version1() {
        std::vector<glm::vec3> newVertices( _vertexPositions.size() , glm::vec3(0,0,0) );
        std::vector<glm::uvec3> newTriangles;

        struct Edge {
            unsigned int a , b;
            Edge( unsigned int c , unsigned int d ) : a( std::min<unsigned int>(c,d) ) , b( std::max<unsigned int>(c,d) ) {}
            bool operator < ( Edge const & o ) const {   return a < o.a  ||  (a == o.a && b < o.b);  }
            bool operator == ( Edge const & o ) const {   return a == o.a  &&  b == o.b;  }
        };
        std::map<Edge, unsigned int> newVertexOnEdge;


        // I) First, compute the even vertices:
        // - you have to find the valence of each vertex
        // - then you compute the position.
        // - Do not change the vertex index! that is convenient to do it this way
        std::vector< unsigned int > evenVertexValence( _vertexPositions.size() , 0 );
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];
            evenVertexValence[ a ]++;
            evenVertexValence[ b ]++;
            evenVertexValence[ c ]++;
        }

        // add the contributions of the neighbors:
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];

            float alpha_a = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[a] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[a] ))) / 64.f;
            float alpha_b = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[b] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[b] ))) / 64.f;
            float alpha_c = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[c] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[c] ))) / 64.f;

            newVertices[ a ] += _vertexPositions[b] * ( alpha_a / (2.f * evenVertexValence[a]) ); // it is divided by two because b will be found in two different triangles
            newVertices[ a ] += _vertexPositions[c] * ( alpha_a / (2.f * evenVertexValence[a]) );

            newVertices[ b ] += _vertexPositions[a] * ( alpha_b / (2.f * evenVertexValence[b]) );
            newVertices[ b ] += _vertexPositions[c] * ( alpha_b / (2.f * evenVertexValence[b]) );

            newVertices[ c ] += _vertexPositions[b] * ( alpha_c / (2.f * evenVertexValence[c]) );
            newVertices[ c ] += _vertexPositions[a] * ( alpha_c / (2.f * evenVertexValence[c]) );
        }

        // add the contributions of the even vertices in the previous level:
        for(unsigned int v = 0 ; v < _vertexPositions.size() ; ++v) {
            float alpha_v = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[v] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[v] ))) / 64.f;

            newVertices[ v ] += (1.f - alpha_v) * _vertexPositions[v];
        }


        // II) Then, compute the odd vertices:
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];


            Edge Eab(a,b);
            unsigned int oddVertexOnEdgeEab = 0;
            if( newVertexOnEdge.find( Eab ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEab = newVertices.size() - 1;
                newVertexOnEdge[Eab] = oddVertexOnEdgeEab;
            }
            else { oddVertexOnEdgeEab = newVertexOnEdge[Eab]; }

            newVertices[ oddVertexOnEdgeEab ] += 3.f * (_vertexPositions[ a ] + _vertexPositions[ b ]) / 16.f + _vertexPositions[ c ] / 8.f;



            Edge Ebc(b,c);
            unsigned int oddVertexOnEdgeEbc = 0;
            if( newVertexOnEdge.find( Ebc ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEbc = newVertices.size() - 1;
                newVertexOnEdge[Ebc] = oddVertexOnEdgeEbc;
            }
            else { oddVertexOnEdgeEbc = newVertexOnEdge[Ebc]; }

            newVertices[ oddVertexOnEdgeEbc ] += 3.f * (_vertexPositions[ b ] + _vertexPositions[ c ]) / 16.f + _vertexPositions[ a ] / 8.f;



            Edge Eca(c,a);
            unsigned int oddVertexOnEdgeEca = 0;
            if( newVertexOnEdge.find( Eca ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEca = newVertices.size() - 1;
                newVertexOnEdge[Eca] = oddVertexOnEdgeEca;
            }
            else { oddVertexOnEdgeEca = newVertexOnEdge[Eca]; }

            newVertices[ oddVertexOnEdgeEca ] += 3.f * (_vertexPositions[ c ] + _vertexPositions[ a ]) / 16.f + _vertexPositions[ b ] / 8.f;



            // set new triangles :
            newTriangles.push_back( glm::uvec3( a , oddVertexOnEdgeEab , oddVertexOnEdgeEca ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , b , oddVertexOnEdgeEbc ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEca , oddVertexOnEdgeEbc , c ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , oddVertexOnEdgeEbc , oddVertexOnEdgeEca ) );
        }


        // after that:
        _triangleIndices = newTriangles;
        _vertexPositions = newVertices;
        recomputePerVertexNormals( );
        recomputePerVertexTextureCoordinates( );
    }








    // THIS VERSION IS ACCEPTABLE. IT REQUIRES ITERATING ONLY ONTO THE TRIANGLES, AND HANDLES BOUNDARIES.
    void subdivideLoop_version2() {
        // Declare new vertices and new triangles. Initialize the new positions for the even vertices with (0,0,0):
        std::vector<glm::vec3> newVertices( _vertexPositions.size() , glm::vec3(0,0,0) );
        std::vector<glm::uvec3> newTriangles;

        struct Edge {
            unsigned int a , b;
            Edge( unsigned int c , unsigned int d ) : a( std::min<unsigned int>(c,d) ) , b( std::max<unsigned int>(c,d) ) {}
            bool operator < ( Edge const & o ) const {   return a < o.a  ||  (a == o.a && b < o.b);  }
            bool operator == ( Edge const & o ) const {   return a == o.a  &&  b == o.b;  }
        };

        std::map< Edge , unsigned int > newVertexOnEdge;             // this will be useful to find out whether we already inserted an odd vertex or not
        std::map< Edge , std::set< unsigned int > > trianglesOnEdge; // this will be useful to find out if an edge is boundary or not
        std::vector< bool > evenVertexIsBoundary( _vertexPositions.size() , false );


        // I) First, compute the valences of the even vertices, and the boundaries:
        std::vector< unsigned int > evenVertexValence( _vertexPositions.size() , 0 );
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];
            evenVertexValence[ a ]++;
            evenVertexValence[ b ]++;
            evenVertexValence[ c ]++;

            trianglesOnEdge[  Edge(a,b) ].insert( tIt );
            trianglesOnEdge[  Edge(c,b) ].insert( tIt );
            trianglesOnEdge[  Edge(a,c) ].insert( tIt );
        }
        // ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
        // IMPORTANT NOTE!
        // If a vertex i is a boundary vertex, then the number of adjacent triangles is not the same as the number of vertices! (draw a picture to see that).
        // Which means that evenVertexValence[i] will be wrong for a boundary vertex i.
        // This is however of no importance in the end, because boundary vertices are handled differently in this version (their treatment does not require the valence of the vertex, see the course material).
        // ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

        // Using the number of triangles adjacent to a given edge, we can find out if the edge is a boundary edge. If it is, then its vertices are boundary vertices as well:
        for( std::map< Edge , std::set< unsigned int > >::const_iterator it = trianglesOnEdge.begin() ; it != trianglesOnEdge.end() ; ++it ) {
            if( it->second.size() == 1 ) {
                // then it is a boundary edge:
                Edge e = it->first;
                evenVertexIsBoundary[ e.a ] = true;
                evenVertexIsBoundary[ e.b ] = true;
            }
        }

        // II) Then, compute the positions for the even vertices: (make sure that you handle the boundaries correctly)
        // add the contributions of the neighbors:
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];

            float alpha_a = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[a] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[a] ))) / 64.f;
            float alpha_b = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[b] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[b] ))) / 64.f;
            float alpha_c = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[c] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[c] ))) / 64.f;

            if(evenVertexIsBoundary[ a ]) {
                if( trianglesOnEdge[ Edge(a,b) ].size() == 1 ) {   newVertices[ a ] += _vertexPositions[b] / 8.f;  }
                if( trianglesOnEdge[ Edge(a,c) ].size() == 1 ) {   newVertices[ a ] += _vertexPositions[c] / 8.f;  }
            }
            else{
                newVertices[ a ] += _vertexPositions[b] * ( alpha_a / (2.f * evenVertexValence[a]) ); // it is divided by two because b will be found in two different triangles
                newVertices[ a ] += _vertexPositions[c] * ( alpha_a / (2.f * evenVertexValence[a]) );
            }

            if(evenVertexIsBoundary[ b ]) {
                if( trianglesOnEdge[ Edge(a,b) ].size() == 1 ) {   newVertices[ b ] += _vertexPositions[a] / 8.f;  }
                if( trianglesOnEdge[ Edge(b,c) ].size() == 1 ) {   newVertices[ b ] += _vertexPositions[c] / 8.f;  }
            }
            else{
                newVertices[ b ] += _vertexPositions[a] * ( alpha_b / (2.f * evenVertexValence[b]) );
                newVertices[ b ] += _vertexPositions[c] * ( alpha_b / (2.f * evenVertexValence[b]) );
            }

            if(evenVertexIsBoundary[ c ]) {
                if( trianglesOnEdge[ Edge(c,b) ].size() == 1 ) {   newVertices[ c ] += _vertexPositions[b] / 8.f;  }
                if( trianglesOnEdge[ Edge(a,c) ].size() == 1 ) {   newVertices[ c ] += _vertexPositions[a] / 8.f;  }
            }
            else{
                newVertices[ c ] += _vertexPositions[b] * ( alpha_c / (2.f * evenVertexValence[c]) );
                newVertices[ c ] += _vertexPositions[a] * ( alpha_c / (2.f * evenVertexValence[c]) );
            }
        }

        // add the contributions of the even vertices in the previous level:
        for(unsigned int v = 0 ; v < _vertexPositions.size() ; ++v) {
            if(evenVertexIsBoundary[ v ]) {
                newVertices[ v ] += 3.f * _vertexPositions[v] / 4.f;
            }
            else {
                float alpha_v = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[v] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[v] ))) / 64.f;
                newVertices[ v ] += (1.f - alpha_v) * _vertexPositions[v];
            }
        }


        // III) Then, compute the odd vertices:
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];


            Edge Eab(a,b);
            unsigned int oddVertexOnEdgeEab = 0;
            if( newVertexOnEdge.find( Eab ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEab = newVertices.size() - 1;
                newVertexOnEdge[Eab] = oddVertexOnEdgeEab;
            }
            else { oddVertexOnEdgeEab = newVertexOnEdge[Eab]; }

            if( trianglesOnEdge[ Edge(a,b) ].size() == 1 ) {
                newVertices[ oddVertexOnEdgeEab ] = (_vertexPositions[ a ] + _vertexPositions[ b ]) / 2.f;
            }
            else {
                newVertices[ oddVertexOnEdgeEab ] += 3.f * (_vertexPositions[ a ] + _vertexPositions[ b ]) / 16.f + _vertexPositions[ c ] / 8.f;
            }



            Edge Ebc(b,c);
            unsigned int oddVertexOnEdgeEbc = 0;
            if( newVertexOnEdge.find( Ebc ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEbc = newVertices.size() - 1;
                newVertexOnEdge[Ebc] = oddVertexOnEdgeEbc;
            }
            else { oddVertexOnEdgeEbc = newVertexOnEdge[Ebc]; }

            if( trianglesOnEdge[ Edge(b,c) ].size() == 1 ) {
                newVertices[ oddVertexOnEdgeEbc ] = (_vertexPositions[ b ] + _vertexPositions[ c ]) / 2.f;
            }
            else {
                newVertices[ oddVertexOnEdgeEbc ] += 3.f * (_vertexPositions[ b ] + _vertexPositions[ c ]) / 16.f + _vertexPositions[ a ] / 8.f;
            }


            Edge Eca(c,a);
            unsigned int oddVertexOnEdgeEca = 0;
            if( newVertexOnEdge.find( Eca ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEca = newVertices.size() - 1;
                newVertexOnEdge[Eca] = oddVertexOnEdgeEca;
            }
            else { oddVertexOnEdgeEca = newVertexOnEdge[Eca]; }

            if( trianglesOnEdge[ Edge(c,a) ].size() == 1 ) {
                newVertices[ oddVertexOnEdgeEca ] = (_vertexPositions[ a ] + _vertexPositions[ c ]) / 2.f;
            }
            else {
                newVertices[ oddVertexOnEdgeEca ] += 3.f * (_vertexPositions[ c ] + _vertexPositions[ a ]) / 16.f + _vertexPositions[ b ] / 8.f;
            }


            // set new triangles :
            newTriangles.push_back( glm::uvec3( a , oddVertexOnEdgeEab , oddVertexOnEdgeEca ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , b , oddVertexOnEdgeEbc ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEca , oddVertexOnEdgeEbc , c ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , oddVertexOnEdgeEbc , oddVertexOnEdgeEca ) );
        }


        // after that:
        _triangleIndices = newTriangles;
        _vertexPositions = newVertices;
        recomputePerVertexNormals( );
        recomputePerVertexTextureCoordinates( );
    }















    // THIS VERSION IS ACCEPTABLE. IT REQUIRES ITERATING ONTO THE TRIANGLES TO BUILD THE VERTEX-VERTEX ADJACENCY LIST, AND HANDLES BOUNDARIES.
    void subdivideLoop_version3() {
        // Declare new vertices and new triangles. Initialize the new positions for the even vertices with (0,0,0):
        std::vector<glm::vec3> newVertices( _vertexPositions.size() , glm::vec3(0,0,0) );
        std::vector<glm::uvec3> newTriangles;

        struct Edge {
            unsigned int a , b;
            Edge( unsigned int c , unsigned int d ) : a( std::min<unsigned int>(c,d) ) , b( std::max<unsigned int>(c,d) ) {}
            bool operator < ( Edge const & o ) const {   return a < o.a  ||  (a == o.a && b < o.b);  }
            bool operator == ( Edge const & o ) const {   return a == o.a  &&  b == o.b;  }
        };

        std::map< Edge , unsigned int > newVertexOnEdge;             // this will be useful to find out whether we already inserted an odd vertex or not
        std::map< Edge , std::set< unsigned int > > trianglesOnEdge; // this will be useful to find out if an edge is boundary or not
        std::vector< std::set< unsigned int > > neighboringVertices( _vertexPositions.size() ); // this will be used to store the adjacent vertices, i.e., neighboringVertices[i] will be the list of vertices that are adjacent to vertex i.
        std::vector< bool > evenVertexIsBoundary( _vertexPositions.size() , false );


        // I) First, compute the valences of the even vertices, the neighboring vertices required to update the position of the even vertices, and the boundaries:
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];

            trianglesOnEdge[  Edge(a,b) ].insert( tIt );
            trianglesOnEdge[  Edge(c,b) ].insert( tIt );
            trianglesOnEdge[  Edge(a,c) ].insert( tIt );

            neighboringVertices[ a ].insert( b );
            neighboringVertices[ a ].insert( c );
            neighboringVertices[ b ].insert( a );
            neighboringVertices[ b ].insert( c );
            neighboringVertices[ c ].insert( a );
            neighboringVertices[ c ].insert( b );
        }

        // The valence of a vertex is the number of adjacent vertices:
        std::vector< unsigned int > evenVertexValence( _vertexPositions.size() , 0 );
        for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
            evenVertexValence[ v ] = neighboringVertices[ v ].size();
        }

        // Using the number of triangles adjacent to a given edge, we can find out if the edge is a boundary edge. If it is, then its vertices are boundary vertices as well.
        // For boundary even vertices, only the neighboring boundary vertices are required to update their position:
        std::vector< std::set< unsigned int > > neighboringVerticesForBoundaryVertices( _vertexPositions.size() );
        for( std::map< Edge , std::set< unsigned int > >::const_iterator it = trianglesOnEdge.begin() ; it != trianglesOnEdge.end() ; ++it ) {
            if( it->second.size() == 1 ) {
                // then it is a boundary edge:
                Edge e = it->first;
                evenVertexIsBoundary[ e.a ] = true;
                evenVertexIsBoundary[ e.b ] = true;

                neighboringVerticesForBoundaryVertices[ e.a ].insert( e.b );
                neighboringVerticesForBoundaryVertices[ e.b ].insert( e.a );
            }
        }

        // II) Then, compute the positions for the even vertices: (make sure that you handle the boundaries correctly)
        for(unsigned int v = 0 ; v < _vertexPositions.size() ; ++v) {
            if( evenVertexIsBoundary[ v ] ) {
                // then we use the mask for the boundary / crease even vertices:
                newVertices[ v ] = 0.75f * _vertexPositions[ v ];
                // you can check that a boundary vertex should have only two neighboring boundary vertices for a manifold mesh (i.e., neighboringVerticesForBoundaryVertices[v].size() should equal 2)
                for( std::set< unsigned int >::const_iterator neighborIt = neighboringVerticesForBoundaryVertices[v].begin() ; neighborIt != neighboringVerticesForBoundaryVertices[v].end() ; ++neighborIt ) {
                    unsigned int neighborIndex = *neighborIt;
                    newVertices[ v ] += (1.f / 8.f) * _vertexPositions[ neighborIndex ];
                }
            }
            else {
                // then we use the mask for the non-boundary even vertices:
                float alpha_v = (40.f - (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[v] )) * (3.f + 2.f * cos( 2.f * M_PI / evenVertexValence[v] ))) / 64.f;
                newVertices[ v ] = (1.f - alpha_v) * _vertexPositions[ v ];
                for( std::set< unsigned int >::const_iterator neighborIt = neighboringVertices[v].begin() ; neighborIt != neighboringVertices[v].end() ; ++neighborIt ) {
                    unsigned int neighborIndex = *neighborIt;
                    newVertices[ v ] += (alpha_v / evenVertexValence[v]) * _vertexPositions[ neighborIndex ];
                }
            }
        }


        // III) Then, compute the odd vertices:
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];


            Edge Eab(a,b);
            unsigned int oddVertexOnEdgeEab = 0;
            if( newVertexOnEdge.find( Eab ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEab = newVertices.size() - 1;
                newVertexOnEdge[Eab] = oddVertexOnEdgeEab;
            }
            else { oddVertexOnEdgeEab = newVertexOnEdge[Eab]; }

            if( trianglesOnEdge[ Edge(a,b) ].size() == 1 ) {
                newVertices[ oddVertexOnEdgeEab ] = (_vertexPositions[ a ] + _vertexPositions[ b ]) / 2.f;
            }
            else {
                newVertices[ oddVertexOnEdgeEab ] += 3.f * (_vertexPositions[ a ] + _vertexPositions[ b ]) / 16.f + _vertexPositions[ c ] / 8.f;
            }



            Edge Ebc(b,c);
            unsigned int oddVertexOnEdgeEbc = 0;
            if( newVertexOnEdge.find( Ebc ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEbc = newVertices.size() - 1;
                newVertexOnEdge[Ebc] = oddVertexOnEdgeEbc;
            }
            else { oddVertexOnEdgeEbc = newVertexOnEdge[Ebc]; }

            if( trianglesOnEdge[ Edge(b,c) ].size() == 1 ) {
                newVertices[ oddVertexOnEdgeEbc ] = (_vertexPositions[ b ] + _vertexPositions[ c ]) / 2.f;
            }
            else {
                newVertices[ oddVertexOnEdgeEbc ] += 3.f * (_vertexPositions[ b ] + _vertexPositions[ c ]) / 16.f + _vertexPositions[ a ] / 8.f;
            }


            Edge Eca(c,a);
            unsigned int oddVertexOnEdgeEca = 0;
            if( newVertexOnEdge.find( Eca ) == newVertexOnEdge.end() ) {
                newVertices.push_back( glm::vec3(0,0,0) );
                oddVertexOnEdgeEca = newVertices.size() - 1;
                newVertexOnEdge[Eca] = oddVertexOnEdgeEca;
            }
            else { oddVertexOnEdgeEca = newVertexOnEdge[Eca]; }

            if( trianglesOnEdge[ Edge(c,a) ].size() == 1 ) {
                newVertices[ oddVertexOnEdgeEca ] = (_vertexPositions[ a ] + _vertexPositions[ c ]) / 2.f;
            }
            else {
                newVertices[ oddVertexOnEdgeEca ] += 3.f * (_vertexPositions[ c ] + _vertexPositions[ a ]) / 16.f + _vertexPositions[ b ] / 8.f;
            }


            // set new triangles :
            newTriangles.push_back( glm::uvec3( a , oddVertexOnEdgeEab , oddVertexOnEdgeEca ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , b , oddVertexOnEdgeEbc ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEca , oddVertexOnEdgeEbc , c ) );
            newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , oddVertexOnEdgeEbc , oddVertexOnEdgeEca ) );
        }


        // after that:
        _triangleIndices = newTriangles;
        _vertexPositions = newVertices;
        recomputePerVertexNormals( );
        recomputePerVertexTextureCoordinates( );
    }













    void subdivideLoop() {
        // subdivideLinear();
        // subdivideLoop_version1();
        // subdivideLoop_version2();
        subdivideLoop_version3();
    }












    // -------------------------------------------------------------------------- //
    // -------------------------------------------------------------------------- //
    //          MESH FILTERING PRACTICAL
    // -------------------------------------------------------------------------- //
    // -------------------------------------------------------------------------- //


    void computeUniformWeights( bool normalizeWeights = true ) {
        _vertex_vertex_weights.clear();
        _vertex_vertex_weights.resize( _vertexPositions.size() );
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];

            _vertex_vertex_weights[a][b] = 1.0;
            _vertex_vertex_weights[a][c] = 1.0;
            _vertex_vertex_weights[b][a] = 1.0;
            _vertex_vertex_weights[b][c] = 1.0;
            _vertex_vertex_weights[c][b] = 1.0;
            _vertex_vertex_weights[c][a] = 1.0;
        }

        // normalize:
        if(normalizeWeights)
            for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
                float sumWeights = 0.f;
                for( std::map< unsigned int , float >::iterator it = _vertex_vertex_weights[v].begin() ; it != _vertex_vertex_weights[v].end() ; ++it ) {
                    float w = it->second;
                    sumWeights += w;
                }
                for( std::map< unsigned int , float >::iterator it = _vertex_vertex_weights[v].begin() ; it != _vertex_vertex_weights[v].end() ; ++it ) {
                    unsigned int v2 = it->first;
                    _vertex_vertex_weights[v][v2] /= sumWeights;
                    // std::cout << _vertex_vertex_weights[v][v2] << "  "; // useless debug
                }
                // std::cout << std::endl; // useless debug
            }
    }



    void computeLaplacianWeights( bool normalizeWeights = true ) {
        _vertex_vertex_weights.clear();
        _vertex_vertex_weights.resize( _vertexPositions.size() );
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int v0 = _triangleIndices[tIt][0];
            unsigned int v1 = _triangleIndices[tIt][1];
            unsigned int v2 = _triangleIndices[tIt][2];

            glm::vec3 p0 = _vertexPositions[v0] , p1 = _vertexPositions[v1] , p2 = _vertexPositions[v2];
            float cotAtp0 = dot( p1-p0 , p2-p0 ) / cross( p1-p0 , p2-p0 ).length();
            float cotAtp1 = dot( p2-p1 , p0-p1 ) / cross( p2-p1 , p0-p1 ).length();
            float cotAtp2 = dot( p1-p2 , p0-p2 ) / cross( p1-p2 , p0-p2 ).length();

            // NOTE:
            // If some triangles are badly shaped, some angles might be larger than 90 degrees, in which case the cotangent weights can be negative, which can be problematic for some applications (it is the case for the filtering for example).
            // We have derived the formula for the cotangent Laplacien using the Finite Elements Method.
            // It can also be derived from other theories, such as, for example, what is called "(Discrete) Exterior Calculus".
            // In this theory, we see that the cotangent weight at a triangle corner appears:
            //   the distance from the circumcenter to the opposite edge midpoint divided by the size of the opposite edge.
            // For badly shaped triangles, the circumcenter leaves the triangle (draw it on paper, the circumcenter is the center of the circumscribed sphere, or also the intersection between the three mediatrices of the edges).
            // One way to "fix" these negative weights, is to clamp the circumcenter inside the triangle, and use the corresponding computations in this setup.
            // You can do it this way:
            if( cotAtp0 < 0.f ) {
                cotAtp0 = 0;
                glm::vec3 fakeCircumCenter = 0.5f * ( p1 + p2 );
                cotAtp0 = ( fakeCircumCenter - 0.5f * ( p2 + p1 )  ).length() /  ( p2 - p1 ).length(); // i.e., = 0
                cotAtp1 = ( fakeCircumCenter - 0.5f * ( p0 + p2 )  ).length() /  ( p0 - p2 ).length();
                cotAtp2 = ( fakeCircumCenter - 0.5f * ( p0 + p1 )  ).length() /  ( p0 - p1 ).length();
            }
            else if( cotAtp1 < 0.f ) {
                cotAtp1 = 0;
                glm::vec3 fakeCircumCenter = 0.5f * ( p0 + p2 );
                cotAtp0 = ( fakeCircumCenter - 0.5f * ( p2 + p1 )  ).length() /  ( p2 - p1 ).length();
                cotAtp1 = ( fakeCircumCenter - 0.5f * ( p0 + p2 )  ).length() /  ( p0 - p2 ).length(); // i.e., = 0
                cotAtp2 = ( fakeCircumCenter - 0.5f * ( p0 + p1 )  ).length() /  ( p0 - p1 ).length();
            }
            else if( cotAtp2 < 0.f ) {
                cotAtp2 = 0;
                glm::vec3 fakeCircumCenter = 0.5f * ( p0 + p1 );
                cotAtp0 = ( fakeCircumCenter - 0.5f * ( p2 + p1 )  ).length() /  ( p2 - p1 ).length();
                cotAtp1 = ( fakeCircumCenter - 0.5f * ( p0 + p2 )  ).length() /  ( p0 - p2 ).length();
                cotAtp2 = ( fakeCircumCenter - 0.5f * ( p0 + p1 )  ).length() /  ( p0 - p1 ).length(); // i.e., = 0
            }

            _vertex_vertex_weights[v0][v1] += cotAtp2;
            _vertex_vertex_weights[v1][v0] += cotAtp2;
            _vertex_vertex_weights[v0][v2] += cotAtp1;
            _vertex_vertex_weights[v2][v0] += cotAtp1;
            _vertex_vertex_weights[v2][v1] += cotAtp0;
            _vertex_vertex_weights[v1][v2] += cotAtp0;
        }

        // normalize:
        if(normalizeWeights)
            for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
                float sumWeights = 0.f;
                for( std::map< unsigned int , float >::iterator it = _vertex_vertex_weights[v].begin() ; it != _vertex_vertex_weights[v].end() ; ++it ) {
                    float w = it->second;
                    sumWeights += w;
                }
                for( std::map< unsigned int , float >::iterator it = _vertex_vertex_weights[v].begin() ; it != _vertex_vertex_weights[v].end() ; ++it ) {
                    unsigned int v2 = it->first;
                    _vertex_vertex_weights[v][v2] /= sumWeights;
                    // std::cout << _vertex_vertex_weights[v][v2] << "  "; // useless debug
                }
                // std::cout << std::endl; // useless debug
            }
    }

    void filterVertexPositions( float s = 1.f , bool filterAlongNormal = false ) {
        std::vector<glm::vec3> barycenters( _vertexPositions.size() , glm::vec3(0,0,0) );
        for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
            for( std::map< unsigned int , float >::iterator it = _vertex_vertex_weights[v].begin() ; it != _vertex_vertex_weights[v].end() ; ++it ) {
                unsigned int v2 = it->first;
                float w = it->second;
                barycenters[v] += w * _vertexPositions[v2];
            }
        }

        if(filterAlongNormal) {
            for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
                glm::vec3 displacement = barycenters[v] - _vertexPositions[v];
                displacement = dot(displacement , _vertexNormals[v]) * _vertexNormals[v]; // constrain the displacement to be along the normal
                _vertexPositions[v] = _vertexPositions[v] + s * displacement;
            }
        }
        else {
            for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
                _vertexPositions[v] = s * barycenters[v] + (1.f - s) * _vertexPositions[v];
            }
        }

        recomputePerVertexNormals();
        recomputePerVertexTextureCoordinates();
    }




    // -------------------------------------------------------------------------- //
    // -------------------------------------------------------------------------- //
    //             MESH SIMPLIFICATION PRACTICAL
    // -------------------------------------------------------------------------- //
    // -------------------------------------------------------------------------- //

    void gridSimplification( unsigned int nBits = 32 ) {
        struct Grid{
            unsigned int nBits;
            Grid() {}

            struct Cell{
                glm::vec3 averagePoint;
                unsigned int nPoints;
                int cellIdx;
                Cell() : averagePoint(0,0,0) , nPoints(0) , cellIdx(-1) {}
                bool empty() const { return nPoints == 0 ; }
            };

            std::vector<Cell> cells;
            glm::vec3 bb , BB; // bounding box

            void allocate(unsigned int n_bits) {
                nBits = n_bits;
                cells.resize( nBits*nBits*nBits );
            }

            void build( std::vector< glm::vec3 > const & _vertexPositions ) {
                // compute bounding box (and enlarge it slightly afterwards):
                bb = glm::vec3(FLT_MAX,FLT_MAX,FLT_MAX) ; BB = glm::vec3(-FLT_MAX,-FLT_MAX,-FLT_MAX);
                for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
                    for(int c = 0 ; c < 3 ; ++c) {
                        bb[c] = std::min<float>(bb[c] , _vertexPositions[v][c] );
                        BB[c] = std::max<float>(BB[c] , _vertexPositions[v][c] );
                    }
                }
                float scalingFactor = 1.1;
                {
                    glm::vec3 bbCenter = (bb + BB) / 2.f;
                    bb = bbCenter + scalingFactor * (bb - bbCenter);
                    BB = bbCenter + scalingFactor * (BB - bbCenter);
                }

                // add all points to the grid:
                for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) {
                    addPoint( _vertexPositions[v] );
                }
            }

            Cell & getCell( unsigned int ix , unsigned int iy , unsigned int iz ) {
                return cells[ ix + nBits * iy + nBits*nBits * iz ];
            }

            Cell & getCellContainingPoint( glm::vec3 const & p ) {
                unsigned int ix = ( unsigned int )( floor( nBits * (p[0] - bb[0]) / (BB[0] - bb[0]) ) );
                unsigned int iy = ( unsigned int )( floor( nBits * (p[1] - bb[1]) / (BB[1] - bb[1]) ) );
                unsigned int iz = ( unsigned int )( floor( nBits * (p[2] - bb[2]) / (BB[2] - bb[2]) ) );
                return getCell(ix,iy,iz);
            }

            void addPoint( glm::vec3 const & p ) {
                Cell & cell = getCellContainingPoint(p);
                cell.averagePoint = ((float)(cell.nPoints) * cell.averagePoint + p) / ((float)(cell.nPoints) + 1.f);
                ++cell.nPoints;
            }

            unsigned int indexNonEmptyCells() {
                int current_idx = 0;
                for( unsigned int cIt = 0 ; cIt < cells.size() ; ++cIt ) {
                    Cell & c = cells[cIt];
                    if( ! c.empty() ) {
                        c.cellIdx = current_idx;
                        ++current_idx;
                    }
                }
                return (unsigned int)(current_idx); // that is the number of non empty cells
            }

            void gatherNonEmptyCells( std::vector<glm::vec3> & newVertexPositions ) {
                for( auto & c : cells ) {
                    if( !c.empty() ) {
                        newVertexPositions[c.cellIdx] = c.averagePoint;
                    }
                }
            }
        };

        Grid g;
        g.allocate(nBits);
        g.build( _vertexPositions );

        // index non-empty cells:
        unsigned int newPointsNumber = g.indexNonEmptyCells();

        // get all new points (the average position of non empty cells):
        std::vector<glm::vec3> newVertexPositions(newPointsNumber);
        g.gatherNonEmptyCells( newVertexPositions );

        // index triangles by looking at which cells its corners fall in:
        std::vector<glm::uvec3> newTriangleIndices;
        for( auto & t : _triangleIndices ) {
            glm::vec3 p0 = _vertexPositions[ t[0] ];
            glm::vec3 p1 = _vertexPositions[ t[1] ];
            glm::vec3 p2 = _vertexPositions[ t[2] ];

            Grid::Cell & c0 = g.getCellContainingPoint( p0 );
            Grid::Cell & c1 = g.getCellContainingPoint( p1 );
            Grid::Cell & c2 = g.getCellContainingPoint( p2 );

            if( c0.cellIdx == c1.cellIdx   ||  c0.cellIdx == c2.cellIdx   ||   c2.cellIdx == c1.cellIdx )
                continue;

            newTriangleIndices.push_back( glm::uvec3(c0.cellIdx , c1.cellIdx , c2.cellIdx) );
        }

        // update the mesh:
        _vertexPositions = newVertexPositions;
        _triangleIndices = newTriangleIndices;

        recomputePerVertexNormals( );
        recomputePerVertexTextureCoordinates( );
    }








    void octreeSimplification( unsigned int maxDepth = 12 , unsigned int maxNumberOfPointsPerLeaf = 10 ) {
        struct Octree{
            glm::vec3 bb , BB; // bounding box
            Octree * children[8]; // children
            unsigned int depth; // root is at depth 0
            std::vector< unsigned int > pointIndices; // non empty only for leaves;
            int cellIdx;
            Octree() {
                depth = 0;
                for(int c = 0 ; c < 8 ; ++c) children[c] = NULL;
                cellIdx = -1;
            }
            bool empty() const { return pointIndices.size() == 0 ; }

            ~Octree () {
                for(int c = 0 ; c < 8 ; ++c){
                    if( children[c] != NULL ){
                        delete children[c];
                        children[c] = NULL;
                    }
                }
            }

            void build( std::vector< glm::vec3 > const & _vertexPositions , std::vector< unsigned int > const & _pointIndices ,
                        unsigned int maxDepth = 8 , unsigned int maxNumberOfPointsPerLeaf = 10 ,
                        bool updateBBOX = true ) {
                if(updateBBOX) {
                    // compute bounding box:
                    bb = glm::vec3(FLT_MAX,FLT_MAX,FLT_MAX) ; BB = glm::vec3(-FLT_MAX,-FLT_MAX,-FLT_MAX);
                    for( unsigned int vIt = 0 ; vIt < _pointIndices.size() ; ++vIt ) {
                        unsigned int v = _pointIndices[vIt];
                        for(int c = 0 ; c < 3 ; ++c) {
                            bb[c] = std::min<float>(bb[c] , _vertexPositions[v][c] );
                            BB[c] = std::max<float>(BB[c] , _vertexPositions[v][c] );
                        }
                    }
                }

                if( depth == maxDepth  ||  _pointIndices.size() <= maxNumberOfPointsPerLeaf ) {
                    // then we stop:
                    pointIndices = _pointIndices;
                }
                else {
                    // otherwise we divide the cell into 8 subtrees:
                    std::vector<  std::vector< unsigned int >  >  childrenPointIndices(8);
                    for( unsigned int vIt = 0 ; vIt < _pointIndices.size() ; ++vIt ) {
                        unsigned int v = _pointIndices[vIt];
                        int iX = ( _vertexPositions[v][0] > (bb[0]+BB[0])/2.f ) ? 1 : 0;
                        int iY = ( _vertexPositions[v][1] > (bb[1]+BB[1])/2.f ) ? 1 : 0;
                        int iZ = ( _vertexPositions[v][2] > (bb[2]+BB[2])/2.f ) ? 1 : 0;
                        childrenPointIndices[ iX + 2 * iY + 4 * iZ ].push_back( v );
                    }
                    for(int c = 0 ; c < 8 ; ++c){
                        children[c] = new Octree();
                        children[c]->depth = depth + 1;
                        children[c]->build( _vertexPositions , childrenPointIndices[c] , maxDepth , maxNumberOfPointsPerLeaf );
                    }
                }
            }

            void indexNonEmptyCells( int & current_idx ) {
                if( empty() ) {
                    for(int c = 0 ; c < 8 ; ++c){
                        if( children[c] != NULL ) children[c]->indexNonEmptyCells( current_idx );
                    }
                }
                else {
                    cellIdx = current_idx;
                    ++current_idx;
                }
            }

            void gatherNonEmptyCells( std::vector< glm::vec3 > const & _VertexPositions , std::vector< int > & newVertexIndices , std::vector< glm::vec3 > & newVertexPositions ) {
                if( empty() ) {
                    for(int c = 0 ; c < 8 ; ++c){
                        if( children[c] != NULL ) children[c]->gatherNonEmptyCells( _VertexPositions , newVertexIndices , newVertexPositions );
                    }
                }
                else {
                    glm::vec3 sumPoints(0,0,0);
                    for( auto v : pointIndices ) {
                        newVertexIndices[ v ] = cellIdx;
                        sumPoints += _VertexPositions[ v ];
                    }
                    newVertexPositions[ cellIdx ] = sumPoints / (float)( pointIndices.size() );

                    // NOTE : HERE, WE USE THE AVERAGE POSITION FOR THE REPRENSENTATIVE POINT IN THE LEAF.
                    // WE HAVE SEEN ALTERNATIVES IN THE LESSON (MEDIAN, MINIMIZER OF THE QEF).
                    // YOU CAN TRY TO MODIFY THIS PART TO SEE THE CHANGES.
                    // Note 2 : for the QEF, you need the normals and a set of weights (for example, the vertex normal and the area around the vertex in the mesh)
                    // and you need to compute the minimizer in a robust manner.
                    // Tricks to be sure that the minimizer is "ok":
                    // - the point should still remain inside the cell. You have the bounding box of the cell, you can project the minimizer inside it just to be sure
                    // - if the quadric is degenerate (determinant = 0), you will not be able to invert it.
                    //   in that case, use the SVD (For example, use the Eigen library) to check that.
                    //   the coordinates of the result along the eigenvectors that are degenerate (corresponding eigenvalue = 0) will be 0 by default (corresponding to the solution with smallest magnitude among all possibilities)
                    //   you can obtain the solution that is closest to (for example) the average point in the cell, by changing the coordinate system and centering it around this point.
                    //   that is a simple, yet effective, trick to use in this context.
                }
            }
        };

        Octree g;
        std::vector< unsigned int > _pointIndices ( _vertexPositions.size() );
        for( unsigned int v = 0 ; v < _vertexPositions.size() ; ++v ) { _pointIndices[v] = v; }

        g.build( _vertexPositions , _pointIndices , maxDepth , maxNumberOfPointsPerLeaf );

        // index non-empty cells:
        int newPointsNumber = 0; g.indexNonEmptyCells( newPointsNumber );

        // get all new points (the average position of non empty cells):
        std::vector< glm::vec3 > newVertexPositions( newPointsNumber );
        std::vector< int > newVertexIndices( _vertexPositions.size() );
        g.gatherNonEmptyCells( _vertexPositions , newVertexIndices , newVertexPositions );

        // index triangles by looking at which cells its corners fall in:
        std::vector<glm::uvec3> newTriangleIndices;
        for( auto & t : _triangleIndices ) {
            int i0 = newVertexIndices[ t[0] ];
            int i1 = newVertexIndices[ t[1] ];
            int i2 = newVertexIndices[ t[2] ];

            if( i0 == i1   ||  i0 == i2   ||   i1 == i2 )
                continue;

            assert( i0 < newPointsNumber );
            assert( i1 < newPointsNumber );
            assert( i2 < newPointsNumber );

            assert( i0 >= 0 );
            assert( i1 >= 0 );
            assert( i2 >= 0 );

            newTriangleIndices.push_back( glm::uvec3( i0 , i1 , i2 ) );
        }

        // update the mesh:
        _vertexPositions = newVertexPositions;
        _triangleIndices = newTriangleIndices;

        recomputePerVertexNormals( );
        recomputePerVertexTextureCoordinates( );
    }




private:
    std::vector<glm::vec3> _vertexPositions;
    std::vector<glm::vec3> _vertexNormals;
    std::vector<glm::vec2> _vertexTexCoords;
    std::vector<glm::uvec3> _triangleIndices;

    std::vector< std::map< unsigned int , float > > _vertex_vertex_weights;

    GLuint _vao = 0;
    GLuint _posVbo = 0;
    GLuint _normalVbo = 0;
    GLuint _texCoordVbo = 0;
    GLuint _ibo = 0;
};

// utility: loader
void loadOFF(const std::string &filename, std::shared_ptr<Mesh> meshPtr);

#endif  // MESH_H
