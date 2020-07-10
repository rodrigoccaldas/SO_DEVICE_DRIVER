#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(){
	int fp;
	fp = open("/dev/seri", O_RDWR);
	char c;
	scanf("%c", &c);
	while (c!=EOF)
	{
		write(fp, &c, 1);
		scanf("%c", &c);
	}
	close(fp);
  return 0;
}
