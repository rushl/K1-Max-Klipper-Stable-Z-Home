# Repeats homing on the Z axis until the resulting stepper position has
# fully stabilized within tolerance, and optionally runs a specific Gcode
# sequence before each attempt.
#
# Copyright (C) 2021 Matthew Lloyd <github@matthewlloyd.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging


class StableZHome:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.before_homing_gcode = gcode_macro.load_template(config, 'gcode')
        self.default_max_retries = config.getint("retries", 20, minval=0)
        self.default_retry_tolerance = \
            config.getfloat("retry_tolerance", 1 / 400., above=1/1000.)
        self.default_window = config.getint("window", 4, minval=3)
        # Register STABLE_Z_HOME command
        self.gcode.register_command(
            'STABLE_Z_HOME', self.cmd_STABLE_Z_HOME,
            desc=self.cmd_STABLE_Z_HOME_help)
    cmd_STABLE_Z_HOME_help = (
        "Repeatedly home Z until the Z stepper position stabilizes")
    def cmd_STABLE_Z_HOME(self, gcmd):
        max_retries = gcmd.get_int('RETRIES', self.default_max_retries,
                                   minval=0)
        retry_tolerance = gcmd.get_float('RETRY_TOLERANCE',
                                         self.default_retry_tolerance,
                                         minval=1/1000.)
        window = gcmd.get_int('WINDOW', self.default_window, minval=3)

        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            raise gcmd.error("Printer not ready")
        kin = toolhead.get_kinematics()

        # Check X and Y are homed first.
        curtime = self.printer.get_reactor().monotonic()
        homed_axes = kin.get_status(curtime)['homed_axes']
        if 'x' not in homed_axes or 'y' not in homed_axes:
            raise gcmd.error("Must home X and Y axes first")

        steppers = kin.get_steppers()
        stepper = None
        for s in steppers:
            if s.get_name().startswith('stepper_z'):
                stepper = s
                break
        if stepper is None:
            raise gcmd.error("No Z steppers found")

        self.gcode.respond_info(
            'Stable Z home: %.4f tolerance, window %d, %d max retries\n'
            % (retry_tolerance, window, max_retries))

        mcu_z_readings = []
        retries = 1
        retry_tolerance += 1e-4  # allow for floating point rounding errors
        while retries <= max_retries:
            try:
                self.gcode.run_script_from_command(
                    self.before_homing_gcode.render())
            except Exception:
                logging.exception("Exception running pre-home script")
                raise self.gcode.error('Pre-home Gcode failed')

            self.gcode.run_script_from_command('G28 Z')

            mcu_position_offset = -stepper.mcu_to_commanded_position(0)
            # self.gcode.respond_info("MCU position offset: %.4f" % mcu_position_offset)

            # self.gcode.respond_info("Commanded position: %.4f" % stepper.get_commanded_position())
            # self.gcode.respond_info("MCU position: %.4f" % stepper.get_mcu_position())
            
            zOffset = self.gcode_move.get_status()['homing_origin'].z
            # self.gcode.respond_info("Z Offset: %.4f" % zOffset)
            
            stepper_pos_offset = 18 + (zOffset * 5)
            stepper_window_range_offset = stepper_pos_offset * 3
            # self.gcode.respond_info("Stepper Window Offset: %.4f" % stepper_window_range_offset)
            
            mcu_pos = stepper.get_commanded_position() + mcu_position_offset
            
            # Safely get the last value if the list is not empty
            last_mcu_pos = mcu_z_readings[-1] if mcu_z_readings else None

            # Check if last_mcu_pos is not None (which means the list was not empty)
            if last_mcu_pos is not None:
                self.gcode.respond_info("stepper_z diff: %.4f" % (mcu_pos - last_mcu_pos - stepper_pos_offset))
                
            mcu_z_readings.append(mcu_pos)
            mcu_z_readings = mcu_z_readings[-window:]
            if len(mcu_z_readings) == window:
                window_range = abs( ( max(mcu_z_readings) - min(mcu_z_readings) ) - stepper_window_range_offset )
            else:
                window_range = None

            window_range_str = \
                '%.4f' % (window_range,) if window_range is not None else '-'
            self.gcode.respond_info(
                'Retry %d: %s position %.4f, window range %s\n'
                % (retries, stepper.get_name(), mcu_pos, window_range_str))

            if window_range is not None and window_range <= retry_tolerance:
                self.gcode.respond_info('Succeeded\n')
                break

            retries += 1

        if retries > max_retries:
            raise self.gcode.error('Max retries exceeded\n')


def load_config(config):
    return StableZHome(config)
