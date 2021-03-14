#https://blog.csdn.net/Mahabharata_/article/details/77585393
#https://blog.csdn.net/iscassucess/article/details/7920079
#ifndef MESH_H
#define MESH_H

#include <glad/glad.h>
#include <vector>
#include <memory>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <map>
#include <set>

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









private:
    std::vector<glm::vec3> _vertexPositions;
    std::vector<glm::vec3> _vertexNormals;
    std::vector<glm::vec2> _vertexTexCoords;
    std::vector<glm::uvec3> _triangleIndices;

    GLuint _vao = 0;
    GLuint _posVbo = 0;
    GLuint _normalVbo = 0;
    GLuint _texCoordVbo = 0;
    GLuint _ibo = 0;
};

// utility: loader
void loadOFF(const std::string &filename, std::shared_ptr<Mesh> meshPtr);

#endif  // MESH_H
