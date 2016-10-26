# coding: utf-8
import imp
import crappy2
import time
import numpy as np

crappy2.blocks.MasterBlock.instances = []  # Init masterblock instances
frequence = 150

class EvalForce(crappy2.links.MetaCondition):
    """
    This class permits to send the proper value for tension according to current step.
    """

    def __init__(self):
        self.speed = float(25)  # mm/min
        self.step_force = 200
        self.temporisation = 3.
        self.step = 1


        # Do not change following
        self.bool_charge = True
        self.inc = 0  # Initialization of step

    def evaluate(self, value):
        if time.time() - t0 > self.temporisation:

            if value['Force(N)'] < self.step_force * self.step and self.bool_charge:
                self.inc += 1
            elif value['Force(N)'] > 0 and not self.bool_charge:
                self.inc -= 1

            elif value['Force(N)'] > self.step_force * self.step:
                print 'PALIER ATTEINT! DECHARGE...'
                self.bool_charge = False

            else:
                print 'Demarrage nouveau palier'
                self.step += 1
                self.bool_charge = True
            V = round(self.inc * self.speed / (frequence * 60 * 0.25), 5)
            print 'Current voltage:', V
            labjack_actuator.set_cmd(V)
        return value


def eval_offset(device, duration):
    timeout = time.time() + duration  # 60 secs from now
    print 'Measuring offset (%d sec), please be patient...' % duration
    offset1 = []
    offset2 = []
    while True:
        chan1, chan2 = device.get_data('all')[1]
        offset1.append(chan1)
        offset2.append(chan2)
        if time.time() > timeout:
            offsets = [-np.mean(offset1), -np.mean(offset2)]
            print 'offsets:', offsets
            break
    return offsets


try:

    # Version avec labjack comme sensor uniquement. On recupere effort et deplacement
    comedi_device = crappy2.sensor.ComediSensor(device='/dev/comedi0', channels=[0, 1], range_num=0, gain=[5, 5000],
                                                offset=[0, 0])
    offset = eval_offset(comedi_device, 3)
    comedi_device = crappy2.sensor.ComediSensor(device='/dev/comedi0', channels=[0, 1], range_num=0, gain=[5, 5000],
                                                offset=offset)
    labjack_actuator = crappy2.actuator.LabJackActuator(channel='TDAC0', gain=1, offset=0)
    labjack_actuator.set_cmd(0)
    # EFFORT ET DEPLACEMENT

    # Blocks
    measurebystep = crappy2.blocks.MeasureByStep(sensor=comedi_device, labels=['t(s)', 'deplacement(mm)','Force(N)'], freq=frequence)
    grapher_force = crappy2.blocks.Grapher(('t(s)', 'Force(N)'), window_pos=(1920, 0), length=0)
    grapher_dep = crappy2.blocks.Grapher(('t(s)', 'deplacement(mm)'), window_pos=(1920, 1080), length=0)
    compacter = crappy2.blocks.Compacter(100)
    saver = crappy2.blocks.Saver('/home/francois/Code/A_Projects/012_commande_magalie/test01.csv', stamp='date')
    # Links
    link_to_compacter = crappy2.links.Link(name='to_compacter', condition=EvalForce())
    link_to_force = crappy2.links.Link(name='to_force')
    link_to_deplacement = crappy2.links.Link(name='to_deplacement')
    link_to_saver = crappy2.links.Link(name='to_saver')

    # Linking
    measurebystep.add_output(link_to_compacter)  # > Mesures
    compacter.add_input(link_to_compacter)
    compacter.add_output(link_to_force)  # > Vers les graphs
    compacter.add_output(link_to_deplacement)  # > Vers les graphs
    compacter.add_output(link_to_saver)

    grapher_force.add_input(link_to_force)
    grapher_dep.add_input(link_to_deplacement)
    saver.add_input(link_to_saver)
    # Blocks


    # measurebystep.add_input(trigger_measures)

    # Suite du programme : commande simple
    raw_input('ready?')

    t0 = time.time()

    for instance in crappy2.blocks.MasterBlock.instances:
        instance.t0 = t0

    for instance in crappy2.blocks.MasterBlock.instances:
        instance.start()  # Waiting for execution

except KeyboardInterrupt:
    for instance in crappy2.blocks.MasterBlock.instances:
        instance.stop()
    labjack_actuator.close()
except Exception:
    raise
