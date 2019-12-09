/*
*      Made for calls on port open/close. Copied from mmc.h
*      Someone will probably put a proper way of doing this into the 
*      kernel at some point.
*/
#ifndef ASMARM_ARCH_UART_H
#define ASMARM_ARCH_UART_H

struct device;

struct pxauart_platform_data {
       int (*open)(struct device *, void *);
       void (*close)(struct device *, void *);
       void *data;     //need some way of passing the attached device's struct in.
};

#endif

