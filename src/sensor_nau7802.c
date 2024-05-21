#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "board/gpio.h" // i2c_read
#include "i2ccmds.h" // i2cdev_oid_lookup

#define ADCO_B0 0x14
#define ADCO_B1 0x13
#define ADCO_B2 0x12

enum {
    AX_HAVE_START = 1<<0, AX_RUNNING = 1<<1, AX_PENDING = 1<<2,
};


struct nau7802{
    struct timer timer;
    struct i2cdev_s *i2c;
    uint8_t data[48];
    uint8_t flags, data_count;
};

static struct task_wake nau7802_wake;

static uint_fast8_t nau7802_event(struct timer *timer)
{
    struct nau7802 *nu = container_of(timer, struct nau7802, timer);
    nu->flags |= AX_PENDING;
    sched_wake_task(&nau7802_wake);
     return SF_DONE;
};


void
command_config_nau7802(uint32_t *args){
    struct nau7802 *nu = oid_alloc(args[0], command_config_nau7802, sizeof(*nu));
    nu->timer.func = nau7802_event;
    nu->i2c = i2cdev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_nau7802, "config_nau7802 oid=%c i2c_oid=%c");


void
command_query_nau7802(uint32_t *args)
{    
    struct nau7802 *nu= oid_lookup(args[0], command_config_nau7802);
    uint16_t val = args[1];
    uint8_t msg[3];

    uint8_t reg1 = ADCO_B0;
    uint8_t reg2 = ADCO_B1;
    uint8_t reg3 = ADCO_B2;
    i2c_read(nu->i2c->i2c_config, sizeof(reg3), &reg3, 1, &msg[0]);
    i2c_read(nu->i2c->i2c_config, sizeof(reg2), &reg2, 1, &msg[1]);
    i2c_read(nu->i2c->i2c_config, sizeof(reg1), &reg1, 1, &msg[2]);
    uint32_t data = (msg[0] << 16) | (msg[1] << 8) | msg[2];

    nau7802_status(nu, args[0], data, args[2]);
    struct gpio_out pin;
    pin = gpio_out_setup(args[2], 0);
    if(data > 100000){
        gpio_out_write(pin, 1);
    }
}
DECL_COMMAND(command_query_nau7802,
             "query_nau7802 oid=%c data=%u pin=%u");

void
nau7802_status(struct nau7802 *nu, uint_fast8_t oid, uint32_t data, uint16_t pin)
{
    sendf("nau7802_status oid=%c data=%u pin=%u"
          , oid, data, pin);
}

void
stop_querying(uint32_t *args){
    sendf("y data=%u", args[3]);
    struct gpio_out pin;
    pin = gpio_out_setup(args[3], 0);
    gpio_out_write(pin, 0);
    struct gpio_in pin1;
    pin1 = gpio_in_setup(args[3], 0);
}
DECL_COMMAND(stop_querying, "stop_nau_querying oid=%c pin=%u");
 

static void
nau7802_report(struct nau7802 *nu, uint8_t oid)
{
    sendf("nau7802_data oid=%c data=%s"
          , oid, "hi :)))");
}

void
nau7802_task(void){
if (!sched_check_wake(&nau7802_wake))
        return;
    uint8_t oid;
    struct nau7802 *nu;
    foreach_oid(oid, nu, command_config_nau7802) {
        uint_fast8_t flags = nu->flags;
        if (!(flags & AX_PENDING)) {
            continue;
        }
        if (flags & AX_HAVE_START) {
            nau7802_status(nu, oid, 0, "P1.0");
        }
        else {
            nau7802_report(nu, oid);
        }
    }
}
DECL_TASK(nau7802_task);