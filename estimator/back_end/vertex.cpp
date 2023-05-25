#include "vertex.h"

namespace backend {
	unsigned long global_vertex_id = 0;

	Vertex::Vertex(int num_dimension, int local_dimension) 
	{
		m_parameters.resize(num_dimension, 1);
		m_localDimension = local_dimension > 0 ? local_dimension : num_dimension;
		m_vertexId = global_vertex_id++;
	}

	Vertex::~Vertex() {}

	void Vertex::Plus(const VecXd& delta)
	{
		m_parameters += delta;
	}
}