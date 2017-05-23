
#ifndef __DEF_H__
#define __DEF_H__

#ifdef __cplusplus
extern "C" {
#endif

//#define 	TRUE 	1   
//#define 	FALSE 	0
//#define true  1
//#define false 0

#define PACKED	__packed

typedef	unsigned int	uint32;
typedef	unsigned int	u32;  // jcs
typedef	unsigned short	uint16;
typedef	unsigned short	u16; // jcs
typedef	unsigned char	uint8;
typedef	unsigned char	u8; // jcs
typedef	signed int		int32;
typedef	signed short	int16;
typedef	signed char		int8;

typedef volatile unsigned int	vuint32;
typedef volatile unsigned short	vuint16;
typedef volatile unsigned char	vuint8;
typedef	volatile signed int		vint32;
typedef	volatile signed short	vint16;
typedef	volatile signed char	vint8;

#ifdef __cplusplus
}
#endif

typedef unsigned char	U8,	UINT8;
typedef unsigned short	U16,UINT16;
typedef unsigned int	U32,UINT32;
typedef char			S8,	SINT8,	INT8;
typedef short			S16,SINT16,INT16;
typedef int			S32,SINT32,INT32;
typedef float			F32,FP32;
typedef double		F64,FP64;

//#define NEW(type)			(type *)malloc(sizeof(type))
//#define NEWS(type,count)		(type *)malloc(count*sizeof(type))
//#define DELETE(pt)			do{free((void*)pt);pt=NULL;}while(0)


#ifndef __cplusplus

#ifndef BOOL
#define BOOL UINT32
#endif

#else

#ifndef BOOL
#define BOOL	UINT32
#define TRUE	1
#define FALSE	0 
#endif

#endif

#endif ///*__DEF_H__*/
