#include "basic_dev.h"

basic_dev::basic_dev(const basic_para_type& basic_para)
    : m_basic_para(basic_para),
      p_bar_addr{}
{

};


basic_para_type basic_dev::get_basic_para(){
    return m_basic_para;
}