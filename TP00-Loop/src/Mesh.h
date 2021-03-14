#ifndef MESH_H
#define MESH_H

#include <glad/glad.h>
#include <vector>
#include <memory>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <map>
#include <set>
#include <string>
#include <math.h>

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

    //void init();
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





    void subdivideLoop() {
        //subdivideLinear();

	// TODO: Implement here the Loop subdivision instead of the straightforward Linear Subdivision.
	// You can have a look at the Linear Subdivision function to take some inspiration from it.
	//
	// A few recommendations / advices (note that the following regards a simple implementation that does not handle boundaries, you can adapt it if you want to handle those):
	// I) start by declaring a vector of new positions "newVertices" and a vector of new triangles "newTriangles". 
	//    Do not mix the new quantities and the old ones.
	//    At the end, replace _vertexPositions by newVertices and _triangleIndices by newTriangles, just as it is done in subdivideLinear().
	//    This will help you writing clean code.
	//    Remember: In the Loop subdivision scheme, a new position (in the output mesh at level k+1) is a linear combination of the old vertices positions (at level k).
	//    So, you should NEVER (!!!!!) have in your code something like: newVertices[ v ] += newVertices[ v_neighbor ] * weight;
	// II) Compute the neighbors of all the even vertices. You can use a structure such as "std::vector< std::set< unsigned int > > vertex_neighbors" for example.
	//    This will give you the valence n of a given even vertex v, and the value of the coefficient alpha_n that you need to use in the computation of the new position for v.
	// III) Compute the new positions for the even vertices. If you compute the even vertices first, you will not be tempted to consider the odd vertices as their neighbors (that would be a -- very common, mistake).
	// IV) Process all triangles, insert the odd vertices, compute their position using the subdivision mask, and create four new triangles per old triangle.
	//    You can get inspiration from subdivideLinear() for that part.
	//
	// Good luck! Do not hesitate asking questions, we are here to help you.
        
        std::vector<glm::vec3> newVertices = _vertexPositions;
        std::vector<glm::uvec3> newTriangles;
        
        std::vector< std::set< unsigned int > > vertex_neighbors(_vertexPositions.size());
        
        for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
            unsigned int a = _triangleIndices[tIt][0];
            unsigned int b = _triangleIndices[tIt][1];
            unsigned int c = _triangleIndices[tIt][2];
        
            vertex_neighbors[a].insert(b);
            vertex_neighbors[a].insert(c);
            vertex_neighbors[b].insert(a);
            vertex_neighbors[b].insert(c);
            vertex_neighbors[c].insert(a);
            vertex_neighbors[c].insert(b);
        }
    
//        unsigned int n = vertex_neighbors[0].size();
//        std::cout << n << std::endl;
        
        for(int v = 0; v < _vertexPositions.size(); ++v)
        {
            unsigned int n = vertex_neighbors[v].size();
            float alpha_n = 1.f/64.f * (40.f - pow((3.f + 2.f * cos(2.f * M_PI / n)),2.f));
            newPosition[v] = (1.f - alpha_n) * _vertexPositions;
            for(auto t vertex_neighbors[v]){
                newPosition[v] += (alpha_n / n) * _vertexPositions[vertex_neighbors[v]];
            }
        }
        
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
                newVertices.push_back( (_vertexPositions[ a ] + _vertexPositions[ b ]) * 3.f / 8.f + _vertexPositions[ c ] * 1.f/ 8.f);
                oddVertexOnEdgeEab = newVertices.size() - 1;
                newVertexOnEdge[Eab] = oddVertexOnEdgeEab;
            }
            else { oddVertexOnEdgeEab = newVertexOnEdge[Eab]; }


            Edge Ebc(b,c);
            unsigned int oddVertexOnEdgeEbc = 0;
            if( newVertexOnEdge.find( Ebc ) == newVertexOnEdge.end() ) {
                newVertices.push_back( (_vertexPositions[ c ] + _vertexPositions[ b ]) * 3.f / 8.f + _vertexPositions[ a ] * 1.f/ 8.f);
                oddVertexOnEdgeEbc = newVertices.size() - 1;
                newVertexOnEdge[Ebc] = oddVertexOnEdgeEbc;
            }
            else { oddVertexOnEdgeEbc = newVertexOnEdge[Ebc]; }


            Edge Eca(c,a);
            unsigned int oddVertexOnEdgeEca = 0;
            if( newVertexOnEdge.find( Eca ) == newVertexOnEdge.end() ) {
                newVertices.push_back( (_vertexPositions[ a ] + _vertexPositions[ c ]) * 3.f / 8.f + _vertexPositions[ b ] * 1.f/ 8.f);
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
