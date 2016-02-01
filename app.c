/*
 * This is a user-space application that reads /dev/sample
 * and prints the read characters to stdout
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/time.h>

/************************/
/*    Program Define    */
/************************/
#define TRUE 1
#define BLINK_AV 1
#define BLINK_DIS 0

/************************/
/*    Global Variables  */
/************************/
int fd = -1;




/***********************************************************/
/*                    main                                 */
/* This the main function, it opens the device driver and  */
/* performs and endless loop triggering the sensor and     */ 
/* reading the distance every 100 milliseconds.            */
/***********************************************************/
int main(int argc, char **argv)
{
  unsigned long c;                                 /* used to print the distance from the closest object */
  char *app_name = argv[0];
  char *dev_name = "/dev/sample";
  unsigned int read_flag = 0;                      /* used for printing the distance */


  /* Open the sample device RD | WR */
    if ((fd = open(dev_name, O_RDWR)) < 0) {
      fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));
      return( EXIT_FAILURE );
    }

    /* Set the driver  to use the LED blinking functionality in function to be executed within workqueue */
    ioctl(fd, BLINK_AV, NULL);

    while (TRUE) {

       write(fd, NULL, sizeof(int) );                                 /* write any value to trigger the device */
       read(fd, &c, sizeof(unsigned long));
       if (c != 0) {
          read_flag = 1;
          printf("distance from the closest object :%f \n", (1.0*c)/58);
        }

       usleep(98000u);
       
       if (!read_flag) {
         read(fd, &c, sizeof(unsigned long));
         if (c != 0)
            printf("distance from the closest object :%f \n", (1.0*c)/58);
       }
       read_flag = 0;

    }


  return( EXIT_SUCCESS );
}
