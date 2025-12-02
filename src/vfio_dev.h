#ifndef VFIO_DEV_H
#define VFIO_DEV_H
#include "basic_dev.h"
class vfio_dev : public basic_dev{
    public:
        vfio_dev(
            const basic_para_type& basic_para )                 ;
        ~vfio_dev()                                             ;
        bool initialize()                           override    ;
        bool map_bar ()                             override    ;
        bool enable_dma()                           override    ;
        bool set_interrupt()                                    ;
        vfio_fd_type get_fds(){return m_fds;}                   ;
    private:
        vfio_fd_type                                m_fds       ;
        interrupt_ptr_type                          p_interrupt ;
        bool _get_group_id()                                    ;
        bool _get_group_fd()                                    ;
        bool _get_container_fd()                                ;
        bool _get_device_fd()                                   ;
        bool _add_group_to_container()                          ;
};

#endif // VFIO_DEV_H
