#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

int main()
{
	int fp;
  ssize_t msg = 0;
  

  fp = open("/dev/seri", O_RDONLY | O_NONBLOCK);
  if(fp == -1)
  {
    printf("CANT OPEN THE FILE\n");
    return -1;
  }
  char *str;
  str = (char *)malloc(sizeof(char)*5);
  msg=read(fp, str, 5);
  if(msg<0){
    printf("NOTHING RECEIVED.\n");
  }else{
  printf("MESSAGE RECEIVED: %s\n", str);
      }
  printf("Exiting\n");

  close(fp);
	return 0;
}
