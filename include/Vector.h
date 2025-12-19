#ifndef VECTOR_H
#define VECTOR_H

#include <vector>

namespace LinAlg
{
    template<typename T>
    class Vector {
        public:
        Vector(int size);
        Vector(std::vector<T>);
    };
} // namespace LinAlg




#endif