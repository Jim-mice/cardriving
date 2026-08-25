#ifndef __HAL_BSP_LOG_H__
#define __HAL_BSP_LOG_H__

/******************** 调试打印信息设置 ************************/
#define DEBUG_ON
#ifdef  DEBUG_ON
	#define  console_log_error(fmt, args...)  	printf("[error][%s] "fmt" \n", __FUNCTION__, ##args)
	#define  console_log_success(fmt, args...)  printf("[ok][%s] "fmt" \n", __FUNCTION__, ##args)
	#define  console_log_info(fmt, args...)  	  printf("[info][%s] "fmt" \n", __FUNCTION__, ##args)
#else
	#define  console_log_error(fmt, args...)  	do{}while(0)
	#define  console_log_success(fmt, args...)  	do{}while(0)
	#define  console_log_info(fmt, args...)  	  do{}while(0)
#endif



#endif /* __HAL_BSP_LOG_H__ */
