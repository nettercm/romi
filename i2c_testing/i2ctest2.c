#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <sched.h>
#include <memory.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <time.h>
#include <memory.h>
#include <math.h>
#include <termio.h>
#include <termios.h>
#include <stdint.h>

#include <linux/i2c-dev.h>

#define I2C_ADDR 0x14


int fd;

int errors = 0;

//inline uint32_t us(void)
uint32_t us(void)
{
	struct timespec ts;
	uint32_t microseconds;
	clock_gettime(CLOCK_MONOTONIC_RAW,&ts);
	microseconds = (uint32_t)ts.tv_sec * 1000000UL;
	microseconds+= (uint32_t)ts.tv_nsec / 1000UL;
	return microseconds;
}


int selectDevice(int fd, int addr, char *name)
{
   int s;
   char str[128];

    s = ioctl(fd, I2C_SLAVE, addr);

    if (s == -1)
    {
       sprintf(str, "selectDevice for %s", name);
       perror(str);
    }

    return s;
}

int writeToDevice(int fd, int reg, int val)
{
   int s;
   char buf[2];

   buf[0]=reg; buf[1]=val;

   s = write(fd, buf, 2);

   if (s == -1)
   {
      perror("writeToDevice");
   }
   else if (s != 2)
   {
      fprintf(stderr, "short write to device\n");
   }
}


int leds(int red, int yellow, int green)
{
      unsigned char buf[4];
      
      buf[0] = 0;
      buf[1]=(unsigned char)red; 
      buf[2]=(unsigned char)yellow; 
      buf[3]=(unsigned char)green;
      if ((write(fd, buf, 4)) != 4)
      {
         //fprintf(stderr, "leds(): Error writing\n");
         errors++;
         return -1;
      }
      return 0;
}

int encoders(unsigned short *left, unsigned short *right)
{
      unsigned char buf[4];
      
      buf[0] = 39;
      if ((write(fd, buf, 1)) != 1)
      {
         //fprintf(stderr, "encoders(): Error writing\n");
         errors++;
         return -1;
      }
      usleep(200);  //320:  less errors than 100
      if ((read(fd, buf, 4)) != 4)
      {
         //fprintf(stderr, "encoders(): Error reading\n");
         errors++;
         return -1;
      }
      
      *left  = *((unsigned short*)(&buf[0]));
      *right = *((unsigned short*)(&buf[2]));
      
      return 0;
}

int main(int argc, char **argv)
{
   unsigned int range;
   int bus;
   int count, b;
   short x, y, z;
   float xa, ya, za;
   unsigned char buf[16];
   int delay;
   unsigned short left,right;
   unsigned short last_left=0,last_right=0;
   int iterations;
   int i;
   uint32_t t1,t2;

   if (argc > 1) bus = atoi(argv[1]);
   else bus = 0;
   
   if (argc > 2) delay = atoi(argv[2]);
   else delay = 100000;

   if (argc > 3) iterations = atoi(argv[3]);
   else iterations = 1000;

   printf("bus=%d, delay=%d, iterations=%d\n",bus,delay,iterations);
   
   sprintf(buf, "/dev/i2c-%d", bus);
   
   if ((fd = open(buf, O_RDWR)) < 0)
   {
      // Open port for reading and writing

      fprintf(stderr, "Failed to open i2c bus /dev/i2c-%d\n", bus);

      exit(1);
   }
   
   selectDevice(fd, I2C_ADDR, "romi");
   
   leds(1,1,1);

   t1 = us();
   
   for(i=0;i<iterations;i++)
   {   
      //leds(0,0,0);
      //usleep(delay);
      //leds(1,1,1);
      //usleep(delay);
      encoders(&left,&right);
      if( (left != last_left) || (right != last_right) )
      {
        last_left = left;
        last_right = right;
        printf("encoders=%5d,%5d\n",left,right);
      }
      if(delay>0) usleep(delay);
   }
   leds(0,0,0);
   
   t2 = us();
   
   printf("dT/iter = %lu   errors=%d\n",(t2-t1)/iterations,errors);
   
   close(fd);
   
   return 0;
}
