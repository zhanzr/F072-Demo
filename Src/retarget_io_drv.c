#include "retarget_io_drv.h"

#if defined(__ARMCC_VERSION)
int stdout_putchar (int ch)
{
	uint8_t c = ch;
	HAL_UART_Transmit(&huart1, &c, 1, 1);
	return ch;
}
#else
int _write(int file, char *data, int len)
{
	HAL_UART_Transmit(&huart1, data, len, len);

	return len;
}
#endif

#ifdef __cplusplus
namespace std {
  struct __FILE
  {
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
  };
  FILE __stdout;
  FILE __stdin;
  FILE __stderr;
  int fgetc(FILE *f)
  {
    /* Your implementation of fgetc(). */
    return 0;
  }
  int fputc(int c, FILE *stream)
  {
		stdout_putchar(c);
		return c;
  }
  int ferror(FILE *stream)
  {
    fputc('E', stream);
    fputc('\n', stream);
		
		return -1;
  }
  long int ftell(FILE *stream)
  {
    fputc('T', stream);
    fputc('\n', stream);
		
		return 0;
  }
  int fclose(FILE *f)
  {
    /* Your implementation of fclose(). */
    return 0;
  }
  int fseek(FILE *f, long nPos, int nMode)
  {
    /* Your implementation of fseek(). */
    return 0;
  }
  int fflush(FILE *f)
  {
    /* Your implementation of fflush(). */    
    return 0;
  }
}
#endif
