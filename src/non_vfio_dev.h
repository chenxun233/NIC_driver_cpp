#ifndef NON_VFIO_DEV_H
#define NON_VFIO_DEV_H
#include "basic_dev.h"
class non_vfio_dev : public basic_dev{
    public:
        non_vfio_dev(
            const basic_para_type& basic_para )                 ;
        ~non_vfio_dev()                                         ;
        bool initialize()                           override    ;
        bool map_bar ()                             override    ;
        bool enable_dma()                           override    ;
        bool remove_ixgbe_driver()                              ;

    private:    
};


#endif // NON_VFIO_DEV_H