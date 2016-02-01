/*
 * sample.c - The simplest loadable kernel module.
 * Intended as a template for development of more
 * meaningful kernel modules.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>

/*************************************/
/*            Program Define         */
/*************************************/
#define LED_ON  1
#define LED_OFF 0


/***************************************************/
/*    Definition of the GPIOs we want to use       */
/***************************************************/
#define TRIGGER 131
#define ECHO    130
#define LED     129


 /***************************************************/
 /*               Device major number               */
 /***************************************************/
static uint module_major = 166;

/***************************************************/
/*                Device name                      */
/***************************************************/
static char * module_name = "sample";


/*************************************************************************/
/*  Device access lock. Only one process can access the driver at a time */
/*************************************************************************/
static int sample_lock = 0;


/***************************************************/
/*                Device Variables                 */
/***************************************************/
static int old_value = 0;                         /* used to detect the falling/rising edge of the ECHO signal */
static int new_value = 0;                         /* used to detect the falling/rising edge of the ECHO signal */



/***************************************************/
/*                Declare the Work Queue           */
/***************************************************/
static struct workqueue_struct *my_wq;

typedef struct {
  struct work_struct my_work;
  struct timeval before, after;
  unsigned long distance;                         /* distance to be passed to the user, after the measurement           */
  int valid;                                      /* used to understadn whether the content of distance is valid or not */
  int state;                                      /* it keeps track of the system current state                         */
  int counter;                                    /* used to blik with the proper period the LED                        */
  int led_state;                                  /* it keeps track of current LED state(on/off)                        */
  unsigned long elapsed_time;                     /* it holds the duration of the ECHO pulse                            */
  int blink_enable;                               /* used to enable the blinking within my_wq_function                  */
} my_work_t;

static my_work_t work;


/***********************************************************/
/*                    invert_LED                           */
/* This function is used to invert the current status      */
/* and value of the LED.                                   */
/***********************************************************/
static void invert_LED(my_work_t *my_work) {

  if (my_work->led_state == LED_ON) {
    gpio_set_value(LED, 0);
    my_work->led_state = LED_OFF;
  }
  else if (my_work->led_state == LED_OFF) {
    gpio_set_value(LED, 1);
    my_work->led_state = LED_ON;
  }

}


/***********************************************************/
/*                    my_wq_function                       */
/* This function represents the actions that should be     */
/* done by the worker thread within the kernel.            */
/***********************************************************/
static void my_wq_function( struct work_struct *work )
{
  struct timeval before, after;
  my_work_t *my_work;

  my_work = (my_work_t *)work;
  before = my_work->before;
  after = my_work->after;
  my_work->elapsed_time = ((after.tv_sec - before.tv_sec)*1000000 + (after.tv_usec - before.tv_usec));
  my_work->valid = 1;                                             /* measurement is now valid and can be read */




  #ifdef SAMPLE_DEBUG
    printk(KERN_INFO "%s : %s distance is equal to %u, elapsed_time %u \n", module_name, __func__, my_work->distance, my->work->elapsed_time);
  #endif

  /* if this feature is enabled also blink the LED */
  if (my_work->blink_enable) {
    /* turn off/on the LED, depending on the distance */
    if (my_work->elapsed_time <(10*58)) {
      gpio_set_value( LED, 1 );
      my_work->state = 1;
      my_work->counter = 1;
      my_work->led_state = 1;
    }
    else if (my_work->elapsed_time >= (10*58) && my_work->elapsed_time < (25*58)) {
      if (my_work->state == 2 && my_work->counter == 2) {
        invert_LED(my_work);
        my_work->counter = 1;
      }
      else if (my_work->state == 2 && my_work->counter < 2) {
        my_work->counter ++;
      }
      else if (my_work->state != 2) {
        my_work->counter = 1;
        my_work->state = 2;
        invert_LED(my_work);
      }
    }
    else if (my_work->elapsed_time >= (25*58) && my_work->elapsed_time < (50*58)) {
      if (my_work->state == 3 && my_work->counter == 3) {
        invert_LED(my_work);
        my_work->counter = 1;
      }
      else if (my_work->state == 3 && my_work->counter < 3) {
        my_work->counter ++;
      }
      else if (my_work->state != 3) {
        my_work->counter = 1;
        my_work->state = 3;
        invert_LED(my_work);
      }
    }
    else if (my_work->elapsed_time >= (50*58) && my_work->elapsed_time < (75*58)) {
      if (my_work->state == 4 && my_work->counter == 4) {
        invert_LED(my_work);
        my_work->counter = 1;
      }
      else if (my_work->state == 4 && my_work->counter < 4) {
        my_work->counter ++;
      }
      else if (my_work->state != 4) {
        my_work->counter = 1;
        my_work->state = 4;
        invert_LED(my_work);
      }
    }
    else if (my_work->elapsed_time >= (75*58) && my_work->elapsed_time < (100*58)) {
      if (my_work->state == 5 && my_work->counter == 5) {
        invert_LED(my_work);
        my_work->counter = 1;
      }
      else if (my_work->state == 5 && my_work->counter < 5) {
        my_work->counter ++;
      }
      else if (my_work->state != 5) {
        my_work->counter = 1;
        my_work->state = 5;
        invert_LED(my_work);
      }
    }
    else if (my_work->elapsed_time >= (100*58) ) {
      if (my_work->state == 6 && my_work->counter == 10) {
        invert_LED(my_work);
        my_work->counter = 1;
      }
      else if (my_work->state == 6 && my_work->counter < 10) {
        my_work->counter ++;
      }
      else if (my_work->state != 6) {
        my_work->counter = 1;
        my_work->state = 6;
        invert_LED(my_work);
      }
    }
  }
}


/***********************************************************/
/*                    ECHO_handler                         */
/* This is the ISR called whenever an event occurs on the  */
/* ECHO pin. It issues also the work to the work queue.    */
/***********************************************************/
static irq_handler_t ECHO_handler( unsigned int irq, struct pt_regs *regs )
{

  new_value = gpio_get_value(ECHO);

  if ( new_value != old_value && new_value == 1) {

          old_value = new_value;
          do_gettimeofday(&(work.before));

  }
  else if ( new_value != old_value && new_value == 0) {

            old_value = new_value;
            do_gettimeofday(&(work.after));

            #ifdef SAMPLE_DEBUG
              printk( KERN_INFO "%s: %s\n", module_name, __func__ );
            #endif

            queue_work( my_wq, (struct work_struct *)&work );

  }

  return (irq_handler_t)IRQ_HANDLED;
}


/***********************************************************/
/*                    sample_open                          */
/* This function is called to open the device driver.      */
/***********************************************************/
static int sample_open(struct inode *inode, struct file *file)
{
  int ret = 0;


   /* One process at a time */
  if (sample_lock > 0)
  {
    ret = -EBUSY;
  }
  else
  {
    sample_lock++;


    /* Increment the module use counter */
    try_module_get(THIS_MODULE);

    #ifdef SAMPLE_DEBUG
      printk( KERN_INFO "%s: %s\n", module_name, __func__ );
    #endif
  }

  return( ret );
}


/***********************************************************/
/*                    sample_release                       */
/* This function is called to close the device driver.     */
/***********************************************************/
static int sample_release(struct inode *inode, struct file *file)
{

   /* Release device */
  sample_lock = 0;


   /* Decrement module use counter */
  module_put(THIS_MODULE);

  #ifdef SAMPLE_DEBUG
    printk( KERN_INFO "%s: %s\n", module_name, __func__ );
  #endif

  return( 0 );
}


/*************************************************************/
/*                    sample_read                            */
/* This function is called to read data from the device.     */
/* It stores in buffer 0 when data are not valid, it means   */
/* data has not been updated since the last call or that     */
/* neither ISR and my_wq_function have been called. It       */
/* returns the new value of elapsed_time when data have been */
/* correctly read from the device.                           */
/*************************************************************/
static ssize_t sample_read(struct file *filp, char *buffer,
       size_t length, loff_t * offset)
{

  #ifdef SAMPLE_DEBUG
    printk( KERN_INFO "%s: %s\n", module_name, __func__ );
  #endif

  if (work.valid) {
    copy_to_user(buffer, &(work.elapsed_time), sizeof(unsigned  long));
    work.valid = 0;
    return 1;
  }
  else if (!work.valid) {
    *buffer = 0;
    return 1;
  }
}


/***********************************************************/
/*                    sample_write                         */
/* This function is called to write into the device.       */
/* Once it is called, it triggers TRIGGER pin with a pulse */
/* of 20 microseconds.                                     */
/***********************************************************/
static ssize_t sample_write(struct file *filp, const char *buffer,
        size_t length, loff_t * offset)
{

  gpio_set_value(TRIGGER, 1);
  udelay(20u);
  gpio_set_value(TRIGGER, 0);

  #ifdef SAMPLE_DEBUG
    printk( KERN_INFO "%s: %s\n", module_name, __func__ );
  #endif

  return( 1 );
}


/************************************************************/
/*                    sample_ioctl                          */
/* It's used to enable the blinking within my_wq_function.  */
/************************************************************/
static ssize_t sample_ioctl(struct inode *inode, struct file *filep,
          const unsigned int cmd, const unsigned long arg)
{
  if (cmd == 1) {
    work.blink_enable = 1;
  }
  else if (cmd == 0) {
    work.blink_enable = 0;
  }
  return 0;
}


/***********************************************************/
/*                    Device operations                    */
/***********************************************************/
static struct file_operations sample_fops = {
  .read = sample_read,
  .write = sample_write,
  .open = sample_open,
  .release = sample_release,
  .ioctl = sample_ioctl
};


/***********************************************************/
/*                    sample_init_module                   */
/* This function inititializes the device.                 */
/***********************************************************/
static int __init sample_init_module(void)
{

   /* Register device */
  int ret;

  ret = register_chrdev(module_major, module_name, &sample_fops);
  if (ret < 0) {
    printk(KERN_INFO "%s: registering device %s with major %d failed with %d\n",
           __func__, module_name, module_major, module_major );
    return( ret );
  }
  else
  {
    printk(KERN_INFO "%s: registering device %s with major %d\n",
           __func__, module_name, module_major );


     /* Reserve gpios ECHO (as input) ,TRIGGER (as output, with default output value set to 1)
        and LED (as output, with default vaule of 0) */
    if( gpio_request( TRIGGER, module_name ) )      /* Check whether TRIGGER is available */
    {
      printk( KERN_INFO "%s: %s unable to get TRIGGER gpio\n", module_name, __func__ );
      ret = -EBUSY;
      return( ret );
    }

    if( gpio_request( ECHO, module_name ) )       /* Check whether ECHO is available */
    {
      printk( KERN_INFO "%s: %s unable to get LED gpio\n", module_name, __func__ );
      ret = -EBUSY;
      return( ret );
    }

    if( gpio_request( LED, module_name ) )      /* Check whether LED is available */
    {
      printk( KERN_INFO "%s: %s unable to get LED gpio\n", module_name, __func__ );
      ret = -EBUSY;
      return( ret );
    }

    if( gpio_direction_input( ECHO ) < 0 )      /* Set ECHO gpio as input */
    {
      printk( KERN_INFO "%s: %s unable to set ECHO gpio as input\n", module_name, __func__ );
      ret = -EBUSY;
      return( ret );
    }

    if( gpio_direction_output( TRIGGER, 1 ) < 0 )   /* Set TRIGGER gpio as output with default value 1 */
    {
      printk( KERN_INFO "%s: %s unable to set TRIGGER gpio as output\n", module_name, __func__ );
      ret = -EBUSY;
      return( ret );
    }


    if( gpio_direction_output( LED, 0 ) < 0 )   /* Set LED gpio as output with default value 0 */
    {
      printk( KERN_INFO "%s: %s unable to set LED gpio as output\n", module_name, __func__ );
      ret = -EBUSY;
      return( ret );
    }

    /* Attach the ISR to ECHO pin */
    if( request_irq( gpio_to_irq( ECHO ),
                                 (irq_handler_t) ECHO_handler,
          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ,
          module_name,
          NULL ) < 0 )
    {
      printk( KERN_INFO "%s: %s unable to register gpio irq for ECHO\n", module_name, __func__ );
      ret = -EBUSY;
      return( ret );
    }

    /* Create the work queue */
    my_wq = create_workqueue( "my_queue" );
    if( my_wq )
    {
      INIT_WORK( (struct work_struct *)&work, my_wq_function );
    }

    /* Variables Init*/
    work.distance = 0;
    work.valid = 0;
    work.state = 0;
    work.counter = 1;
    work.led_state = LED_OFF;
    work.blink_enable = 0;    /*  by default not enabled  */
  }

  return( ret );
}


/***********************************************************/
/*                    sample_cleanup_module                */
/* This function called to clean module resources and      */
/* unregister.                                             */
/***********************************************************/
static void __exit sample_cleanup_module(void)
{

   /* Free irq */
  free_irq( gpio_to_irq( ECHO ), NULL );


   /* Release the gpios */
  gpio_free( ECHO );
  gpio_free( TRIGGER );
  gpio_free( LED );


   /* Unregister device */
  unregister_chrdev(module_major, module_name);

  printk(KERN_INFO "%s: unregistering %s done\n", __func__, module_name );
}

module_init(sample_init_module);
module_exit(sample_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrea Floridia, s224906");
MODULE_DESCRIPTION("Device Driver Example 1");
