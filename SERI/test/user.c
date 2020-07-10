#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>

int main()
{

	int fp1, fp2;
	char c;


	fp1 = open("/dev/seri", O_RDWR);
	fp2 = open("/dev/seri", O_RDWR);
	if (fp2 < 0)
	{
		printf("Could not open 2nd file while 1st was open!\n");
	}

	close(fp1);
	close(fp2);

	return 0;

}
