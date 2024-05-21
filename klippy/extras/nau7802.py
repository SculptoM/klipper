from . import bus
import logging


NAU7802_ADDR = 0x2A


#NAU7802 registers
NAU7802_REGS = {
'PU_CTRL': 0x00, 
'CTRL1': 0x01,
'CTRL2': 0x02,
'OCAL1_B2': 0x03,
'OCAL1_B1': 0x04,
'OCAL1_B0': 0x05,
'GCAL1_B3': 0x06,
'GCAL1_B2': 0x07,
'GCAL1_B1': 0x08,
'GCAL1_B0': 0x09,
'I2C_CTRL': 0x11,
'ADC0_B2': 0x12,
'OTP_B1': 0x15,
'BYPASS': 0x1B,
'REV' : 0x1F
}

#Bits within PU_CTRL register
NAU7802_PU_CTRL_RR = 0
NAU7802_PU_CTRL_PUD = 1
NAU7802_PU_CTRL_PUA = 2
NAU7802_PU_CTRL_PUR = 3
NAU7802_PU_CTRL_CS = 4
NAU7802_PU_CTRL_CR = 5
NAU7802_PU_CTRL_OSCS = 6
NAU7802_PU_CTRL_AVDDS = 7

class NAU7802:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        #self.gcode.register_command("NAU7802_READ_ADC", self.cmd_make_measurement, "reads adc value from NAU7802")
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=NAU7802_ADDR, default_speed=400000)
        self.mcu = self.i2c.get_mcu()
        self.oid = self.mcu.create_oid()
        self.z_endstop_pin = None

        self.mcu.register_config_callback(self._build_config)
        self.mcu.register_response(self._handle_nau7802_data, "nau7802_data", self.oid)
        self.mcu.register_response(self._handle_nau7802_data, "y", self.oid)
        #params = self.mcu.register_response(self._handle_nau7802_data, "nau7802_data", self.oid)
        #logging.info(params)

        logging.info(self.oid)

        self.raw_samples = []
        self.report_time = .1
        self.stepper = None
        self.temp = self.min_temp = self.max_temp = self.humidity = 0
        self.chip_registers = NAU7802_REGS
        self.printer.add_object("nau7802" + self.name, self)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.time = None


        self.printer.register_event_handler("homing:home_rails_begin", self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                       self._handle_home_rails_end)

    def _build_config(self):
        cmdqueue = self.i2c.get_command_queue()
        self.mcu.add_config_cmd("config_nau7802 oid=%d i2c_oid=%d" %(self.oid, self.i2c.get_oid()))
        #Its not end at all, its just asking for quering and response
        self.query_nau7802_cmd = self.mcu.lookup_query_command(
            "query_nau7802 oid=%c data=%u pin=%u",
            "nau7802_status oid=%c data=%u pin=%u", oid=self.oid, cq=cmdqueue)
        #this is to end the querying
        self.query_nau7802_end_cmd = self.mcu.lookup_command("stop_nau_querying oid=%c pin=%u")

    def _handle_home_rails_begin(self, homing_state, rails):
        self.sample_timer = self.printer.get_reactor().register_timer(self.make_measurement)
        logging.info("homing started")
        logging.info("self oid: %d" %self.oid)
        logging.info(rails)
        for rail in rails:
            if(rail.get_steppers()[0].get_name()=='stepper_z'):
                stepper = rail.get_steppers()[0]
                logging.info(stepper)
                logging.info(rail)
                logging.info(rail.endstop_pin)
                self.z_endstop_pin = rail.endstop_pin
                self.start_checks()

    def _handle_home_rails_end(self, homing_state, rails):
        for rail in rails:
            if (rail.get_steppers()[0].get_name() == 'stepper_z'):
                logging.info("Hominng ended!! :)))))")
                logging.info(self.z_endstop_pin)
                #self.query_nau7802_end_cmd.send([self.oid, str(self.z_endstop_pin)])
                self.stop_checks()

    def handle_connect(self):
        self._init_nau7802()

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def _handle_nau7802_data(self, params):
        self.raw_samples.append(params)
    
    def setup_callback(self, cb):
        self._callback = cb
    
    def get_report_time_delta(self):
        return self.report_time

    def get_temp(self):
        return self.temp

    def make_measurement(self, eventtime):
        logging.info(self.z_endstop_pin)
        params = self.query_nau7802_cmd.send([self.oid, self.temp, str(self.z_endstop_pin)], 0, 0)
        logging.info(params)
        self.temp = params['data']
        logging.info(self.temp)
        self.reactor.pause(self.reactor.monotonic() + 0.1)
        return eventtime + 1.

    def _init_nau7802(self):
        self.write_register('PU_CTRL', [0x01]) #reset
        self.write_register('PU_CTRL', [0x02]) #PUD ON
        self.reactor.pause(self.reactor.monotonic()+.20)
        logging.info(bin(self.read_register('PU_CTRL', 1)[0]))
        self.write_register('CTRL1', [0x23]) #gains
        self.write_register('PU_CTRL', [0x1E])  # PUD; PUA; PUR; CS << 1
        logging.info(bin(self.read_register('PU_CTRL', 1)[0]))
        self.reactor.pause(self.reactor.monotonic() + .5)
        logging.info(type(self.temp))
        logging.info("nau7802: successfully initialized, initial temp: " +
                         "%.3f, humidity: %.3f"%(self.temp, self.humidity))

    def stop_checks(self):
        logging.info("Checks done")
        if self.sample_timer is None:
            return
        self.temp = 0
        self.reactor.unregister_timer(self.sample_timer)
    def start_checks(self):
        
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)
        logging.info("Checks started")
        curtime = self.reactor.monotonic()
        return True
    
    def get_status(self, eventtime):
        return {
            'temp': round(self.temp, 2),
            'humidity': self.humidity,
        }

    def read_register(self, reg_name, read_len):
        # read a single register
        regs = [self.chip_registers[reg_name]]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])
    
    def get_bit(self, reg_name, bit_pos):
        reg = self.read_register(reg_name, 1)[0] #integer in decimal
        bits = [int(d) for d in str(bin(reg))[2:]] #make a list from bin
        if bit_pos>len(bits):
            return 0
        else:
            return bits[7-bit_pos]
                  
    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = self.chip_registers[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)
    
def load_config(config):
    pheater = config.get_printer().lookup_object("heaters")
    pheater.add_sensor_factory("NAU7802", NAU7802)