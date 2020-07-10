#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main()
{
	int fp, cont=0;
  ssize_t error = 0;
  char c;
    	fp = open("/dev/serp",O_RDWR);
    	if(fp == -1)
    	{
    		printf("CANT OPEN THE FILE U WANT\n");
    		return -1;
    	}
	
	error = read(fp, &c, 1);
	while(error!=0)
	{
		printf("%c", c);
		error = read(fp, &c, 1);
	}
	close(fp);
	return 0;
}
