#include <multimap>
#include <vector>

typedef unsigned int index_t;
struct Edge {
  unsigned hex : 32;
  unsigned from : 3;
  unsigned to : 3;
};
struct QuarterEdgeMesh {
  std::vector<index_t> hexahedra;
  std::vector<Edge> twin;
  std::vector<Edge> mirror;

  /// Given the edge a->b find the next edge b->c and return c.
  /// a, b, c are serial indices inside a hexahedron as seen in
  /// the figure above
  static index_t nextEdgeInHexahedron(index_t a, index_t b) {
    index_t r;
    index_t i = ( (b ^ (b >> 1) ^ (b >> 2)) & 1) != 0;
    switch(b ^ a)
    {
      case 1: r = i ? b ^ 2 : b ^ 4; break;
      case 2: r = i ? b ^ 4 : b ^ 1; break;
      case 4: r = i ? b ^ 1 : b ^ 2; break;
      default: assert("only one bit should be different" == 0);
    }
    return r;
  }

  static index_t serialOf(Edge e){
    return (e.hex << 6) | (e.from << 3) | e.to;
  }

  /// First edge in the hexahedron that goes from
  Edge firstEdgeOfHex(index_t hex){
    Edge e = { hex, 0, 1 };
    return e;
  }
  Edge nextEdge(Edge e) {
    Edge f = { e.hex, e.to, nextEdgeInHexahedron(e.from, e.to) };
    return f;
  }

  /// The next edge has most of the logic for finding the other
  /// point, we just have to XOR with the changing bit to find the other
  /// side.
  Edge prevEdge(Edge e){
    Edge f = { e.hex, nextEdgeInHexahedron(e.from, e.to) ^ (e.from ^ e.to), e.from };
    return f;
  }

  Edge twinEdge(Edge e){
    return twin[serialOf(e)];
  }

  Edge mirrorEdge(Edge e){
    return mirror[serialOf(e)];
  }

  void addHexahedron(index_t *corners){
    for(int i = 0; i < 8; i++) hexahedra.push_back(corners[i]);
  }

  void rebuildConnectivity() {
    std::multimap<std::pair<index_t,index_t>, Edge> edges;

    /// Go through all the faces by the way of enumerating
    /// the cross edges, as in 1->2, 0->3, etc
    /// But out of the 4 possible cross edges we should only pick
    /// one out of four by picking the one edge where the vertex has
    /// the lowest value.
    ///
    for(size_t i = 0; i < hexahedra.size() / 8; i++)
      for(int a = 0; a < 8; a++)
        for(int bi = 0; bi < 3; bi++)
        {
          int b = a ^ (1 << bi);
          index_t va = hexahedra[i * 8 + a], vb = hexaherda[i * 8 + b];
          Edge e = { i, a, b };
          edges.insert(std::pair<std::pair<index_t,index_t>,Edge> >(std::pair<index_t,index>(va,vb), e));
        }
  }
};

