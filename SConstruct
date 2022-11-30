import os

cmd_options = {
    'board'     :          ['openmote-b'],
	'project'   :          [
# TODO Add comma separated projects list here
		'template',
# Tests
		'test-ds18b20',
		'test-modbus-rtu-master',
		'test-radio-at86rf215-tx',
		'test-rs485-rx',
		'test-spi',
		'test-aes',
		'test-flir',
		'test-modbus-rtu-slave',
		'test-pwm',
		'test-radio-cc2538',
		'test-serial',
		'test-t6615',
		'test-board',
		'test-gpio',
		'test-mote-serial-helper',
		'test-radio-at86rf215',
		'test-radiotimer',
		'test-sht20',
		'test-timer',
		'test-crc',
		'test-ina226',
		'test-openmote',
		'test-radio-at86rf215-rx',
		'test-rendezvous',
		'test-sleeptimer',
		'test-uart',
		'test-bme280',
		'test-opt3001',

	],
    'compiler'  :          ['gcc'],
    'verbose'   :          ['0','1']
}

def validate_option(key, value, env):
    if key not in cmd_options:
        raise ValueError("Unknown switch {0}.".format(key))
    if value not in cmd_options[key]:
        raise ValueError("Unknown {0} \"{1}\". Options are {2}.\n\n".format(key,value,','.join(cmd_options[key])))

cmd_vars = Variables()
cmd_vars.AddVariables(
    (
        'board',                                           # key
        '',                                                # help
        None,                                              # default
        validate_option,                                   # validator
        None,                                              # converter
    ),
    (
        'project',                                         # key
        '',                                                # help
        None,                                              # default
        validate_option,                                   # validator
        None,                                              # converter
    ),
    (
        'bootload',                                        # key
        '',                                                # help
        '',                                                # default
        None,                                              # validator
        None,                                              # converter
    ),
    (
        'compiler',                                        # key
        '',                                                # help
        cmd_options['verbose'][0],                         # default
        validate_option,                                   # validator
        None,                                              # converter
    ),
    (
        'params',                                          # key
        '',                                                # help
        '',                                                # default
        None,                                              # validator
        None,                                              # converter
    ),
    (
        'verbose',                                         # key
        '',                                                # help
        cmd_options['verbose'][0],                         # default
        validate_option,                                   # validator
        int,                                               # converter
    )
)

# Define default environment to support GCC
env = DefaultEnvironment(ENV = os.environ, tools=['cc', 'c++', 'ar', 'gnulink'], variables = cmd_vars)

Export('env')

env.SConscript(
    'SConscript',
    exports = ['env'],
)
