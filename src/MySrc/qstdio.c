#include "qstdio.h"
#include <stdbool.h>

//#include "define.h"
//#include "logger.h"
#include <stdint.h>

int qscanf(const char* str, const char* fmt, ...)
{
	char c;
	char p;
	char t;
	uint8_t count;
	int tempb;
	bool qverbose;
	va_list arg;
	union
	{
		uint8_t* b;
		char* c;
		int* d;
		const char** p;
	} ptr;

	p = 0;
	count = 0;
	va_start(arg, fmt);
	//qverbose = *fmt != '^';
	qverbose = false;

	if (qverbose)
	{
		_logs("qscanf");
		_logs(str);
		_logs(fmt);
	}

	if (*fmt == '^')
	{
		fmt++;
	}

	while ((c = *fmt++) != 0)
	{
		if (qverbose) _logt(c);

		switch (c)
		{
			case '%':
			t = *fmt++;
			
			switch (t)
			{
				case 'p':
					ptr.p = va_arg(arg, const char**);
					if (ptr.p) *ptr.p = str;

				case 'e':
					count++;
					break;

				case 'b':
				case 'd':
				{
					tempb = 0;
					while ((c = *str) != 0)
					{
						if (c < '0' || c > '9') break;

						tempb *= 10;
						tempb += c - '0';
						str++;
					}

					if (t == 'b')
					{
						ptr.b = va_arg(arg, uint8_t*);
						if (ptr.b) *ptr.b = (uint8_t)tempb;
					}
					else
					{
						ptr.d = va_arg(arg, int*);
						if (ptr.d) *ptr.d = tempb;
					}
					
					count++;
					break;
				}
				case 's':
				{
					ptr.c = va_arg(arg, char*);
					// p contains previous symbol, find string between these symbols
					p = *fmt;
					do
					{
						c = *str;
						if (c == p || c == 0)
						{
							break;
						}

						str++;
						
						if (ptr.c) *ptr.c++ = c;
					} while (c != 0);
					
					if (ptr.c) *ptr.c = 0;
					count++;
					break;
				}
			}
			break;
			
			default:
			if (c != *str)
			{
				if (qverbose)  _logt('<');
				return count;
			}

			p = c;
			str++;
		}
	}
	
	va_end(arg);

	if (qverbose)
	{
		_logt('*');
		_logt('\r');
		_logt('\n');
	}
	
	return count;
}

static uint8_t qprintu(char* str, unsigned long n, int system)
{
	uint8_t d;
	uint8_t c = 0;

	do
	{
		d = n % system;
		n /= system;

		*str++ = d + (d < 10 ? '0' : 'A' - 10);
		c++;
	} while (n != 0);

	return c;
}

static void qrev(char* start, uint8_t count)
{
	char c;

	while (count > 1)
	{
		c = start[0];
		start[0] = start[count - 1];
		start[count - 1] = c;

		count--;
		count--;
		start++;
	}
}

char* qprintfv(char* str, const char* fmt, va_list args)
{
	char c;
	char t;
	char* ps;
	uint8_t len;
	
	while ((c = *fmt++) != 0)
	{
		switch (c)
		{
			case '%':
			t = *fmt++;
			
			switch (t)
			{
				case 'b':
				case 'd':
				case 'l':
				case 'x':
				{
					len = qprintu(str, t == 'l' ? va_arg(args, unsigned long) : va_arg(args, unsigned int), t == 'x' ? 16 : 10);
					qrev(str, len);
					str += len;
					break;
				}
				case 's':
				{
					ps = va_arg(args, char*);
					while ((c = *ps++) != 0)
					{
						*str++ = c;
					}
					
					break;
				}
				case '#':
				{
					ps = va_arg(args, char*);
					len = va_arg(args, unsigned int);
					while ((c = *ps++) != 0 && len-- > 0)
					{
						if (c == '#') break;
						*str++ = c;
					}
					
					break;
				}
				//case '%':
				//	*str++ = '%';
				break;
			}
			break;
			
			default:
				*str++ = c;
			break;
		}
	}
	
	*str = 0;
	return str;
}

char* qprintf(char* str, const char* fmt, ...)
{
	va_list args;
	char* r;

	va_start(args, fmt);
	r = qprintfv(str, fmt, args);
	va_end(args);

	return r;
}
