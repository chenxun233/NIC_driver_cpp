# Introduction
Inspired by https://github.com/emmericp/ixy, which is a simple C-based user space NIC driver, this project is a C++ realization of it.
By rebuilding the project in C++, it is better for beginners to understand the hierarchy and workflow of user-space NIC driver.
# Content
This project does not realize all the functions provided in https://github.com/emmericp/ixy yet. It only supports VFIO based NIC. Also, in terms of example applications, only data sending is realized, which is from `ixy-pktgen.c` in `ixy/src/app`
# Features
1. Clearer structure. The **Ring buffer**, **Memory Pool** and the **device** itself are all decoupled.
2. Future extension. `BasicDev`, `RxRingBuffer` and `TxRingbufer` are all abstract classes. This makes future extension available.
3. Better Naming. Some functions and variables are renamed for better understanding.
# Hierarchy illustration
![The hierarchy of the classes in this project](hierarchy.png)
1. `BasicDev` is an abstract class. In this project, `Intel82599Dev` is a realization of it in Intel NIC. A new realization can be done in the future. (For example, FPGA-based NIC driver)
2. `DMAMemoryAllocator` is a helper class, which is implemented using singleton pattern. The function is to allocate DMA-enabled memory.
3. `DMAMemoryPool` is a DMA-memory based memory pool.
4. `RxRingBuffer` and `TxRingBuffer` are two abstract classes.
5. `IXGBE_RxRingBuffer` and `IXGBE_TxRingBuffer` are the realization of the two abstract ring buffer classes. Both of them contain `DMAMemoryPool`. 
