// Minimal stand-ins for the QNX headers, used ONLY by `make check-syntax` so the QNX sources
// can be compile-checked on a non-QNX host. They are not a QNX emulation and are never linked.
#ifndef QNX_STUB_COMMON_H
#define QNX_STUB_COMMON_H
#include <errno.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

#define _DCMD_MISC 1
#define __DIOF(c, s, t) (((c) << 8) | (s))
#define __DIOT(c, s, t) (((c) << 8) | (s) | 0x1000)
#define __DION(c, s)    (((c) << 8) | (s) | 0x2000)
int devctl(int fd, int dcmd, void *dev_data_ptr, size_t n_bytes, int *dev_info_ptr);

#ifndef EOK
#define EOK 0
#endif
#define S_IFNAM 0x5000
#define _FTYPE_ANY 0
#define _RESMGR_DEFAULT (-2)
#define _IO_XTYPE_MASK 0xffu
#define _IO_XTYPE_NONE 0u
#define _RESMGR_CONNECT_NFUNCS 1
#define _RESMGR_IO_NFUNCS 1
#define MSG_FLAG_ALLOC_PULSE 1
#define MSG_FLAG_SIDE_CHANNEL 2

struct _iofunc_attr { int dummy; };
struct _iofunc_ocb  { long offset; };
typedef struct _iofunc_attr iofunc_attr_t;
typedef struct { int dummy; void *iov; } resmgr_context_t;
typedef struct { struct { uint32_t xtype; uint32_t nbytes; } i; } io_write_t;
typedef struct { struct { uint32_t xtype; uint32_t nbytes; } i; } io_read_t;
typedef struct { struct { int dcmd; uint32_t nbytes; } i; struct { int ret_val; int pad; } o; } io_devctl_t;
typedef struct { int dummy; } message_context_t;
typedef struct { int dummy; } dispatch_t;
typedef struct { int dummy; } dispatch_context_t;
typedef struct { int nparts_max; int msg_max_size; } resmgr_attr_t;
typedef struct { int dummy; } resmgr_connect_funcs_t;
typedef struct {
    int (*write)(resmgr_context_t *, io_write_t *, struct _iofunc_ocb *);
    int (*read)(resmgr_context_t *, io_read_t *, struct _iofunc_ocb *);
    int (*devctl)(resmgr_context_t *, io_devctl_t *, struct _iofunc_ocb *);
} resmgr_io_funcs_t;

#define _IO_WRITE_GET_NBYTES(m) ((m)->i.nbytes)
#define _IO_READ_GET_NBYTES(m)  ((m)->i.nbytes)
#define _IO_SET_WRITE_NBYTES(c, n) ((void)(c), (void)(n))
#define _IO_SET_READ_NBYTES(c, n)  ((void)(c), (void)(n))
#define _RESMGR_NPARTS(n) (n)
#define _RESMGR_PTR(c, p, n) ((void)(c), (void)(p), (void)(n), 0)
#define _DEVCTL_DATA(x) ((void *)(&(x) + 1))
#define SETIOV(iov, a, l) ((void)(iov), (void)(a), (void)(l))
#define SIGEV_PULSE_INIT(ev, coid, prio, code, val) ((void)(ev), (void)(coid), (void)(prio), (void)(code), (void)(val))

int iofunc_write_verify(resmgr_context_t *, io_write_t *, struct _iofunc_ocb *, void *);
int iofunc_read_verify(resmgr_context_t *, io_read_t *, struct _iofunc_ocb *, void *);
int iofunc_devctl_default(resmgr_context_t *, io_devctl_t *, struct _iofunc_ocb *);
int resmgr_msgget(resmgr_context_t *, void *, size_t, size_t);
void iofunc_func_init(int, resmgr_connect_funcs_t *, int, resmgr_io_funcs_t *);
void iofunc_attr_init(iofunc_attr_t *, int, void *, void *);
dispatch_t *dispatch_create(void);
int resmgr_attach(dispatch_t *, resmgr_attr_t *, const char *, int, unsigned, resmgr_connect_funcs_t *, resmgr_io_funcs_t *, iofunc_attr_t *);
int pulse_attach(dispatch_t *, int, int, int (*)(message_context_t *, int, unsigned, void *), void *);
int message_connect(dispatch_t *, int);
dispatch_context_t *dispatch_context_alloc(dispatch_t *);
dispatch_context_t *dispatch_block(dispatch_context_t *);
int dispatch_handler(dispatch_context_t *);
#endif
