#ifndef IMPLEMENTATION_H
#define IMPLEMENTATION_H
#include "vfio_dev.h"
#include "non_vfio_dev.h"

typedef std::unique_ptr<basic_dev> dev_ptr_type;

dev_ptr_type create_device(
        const basic_para_type& basic_para
);

#endif // IMPLEMENTATION_H