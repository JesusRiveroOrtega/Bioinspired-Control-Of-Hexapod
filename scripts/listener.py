#!/usr/bin/env python3
# license removed for brevity
# PKG = 'numpy_tutorial'
# import roslib; roslib.load_manifest(PKG)
from __future__ import division
import rospy
from std_msgs.msg import String
import numpy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int64
from hexapodo.msg import Num

DIST = 0
ANGL = 0
input_counter = 0
#archivo = open("vectores_entrada.txt", "w")
distance = 0
angle = 0
t = 0
pub_enable = 0
dist_sav = []
dir_sav = []

def cir_conv(fltr, inputs):
    n = len(fltr)
    conv = numpy.convolve(fltr, numpy.hstack((inputs, inputs, inputs)))

    conv = conv[int(numpy.fix(numpy.true_divide(n, 2)) + len(inputs)): int(numpy.fix(numpy.true_divide(n, 2)) + 2 * len(inputs))]
    return conv


def parab_int(Rm1, Rmax, Rp1, MaxX, Del):
    Num = Del * (Rp1 - Rm1)
    Den = 2 * (Rp1 + Rm1 - 2 * Rmax)
    if Den == 0:
        return 0
    else:
        return (-1 * numpy.true_divide( Num, Den) + MaxX)


def max_fir_neu(Neuron):
    max_fir = numpy.amax(Neuron)
    index = 0
    while(True):
        if max_fir == Neuron[index]:
            return [max_fir, index]
            break
        index += 1

def Enable_publish_vsum(data_esc):
    global pub_enable
    pub_enable = data_esc.data


def Trajectory_integration_input(data):
    global ANGL, DIST, t
    t = 0
    input_array = data.num
    ANGL = input_array[1]
    DIST = input_array[0]
    print([ANGL,DIST])

def VectorSum():

    global ANGL, DIST, distance, angle, t, pub_enable, dir_sav, dist_sav
    rospy.init_node('VectorSum', anonymous=True)

    pub = rospy.Publisher('/VectorSum', Num, queue_size=10)

    rospy.Subscriber("/traj_integration_input", Num, Trajectory_integration_input)
    rospy.Subscriber("/enable_publish_vsum", Int64, Enable_publish_vsum)

    rate = rospy.Rate(500) # 10hz

    n = 24
    angle = 0
    distance = 0

    angle_acum = angle

    pref_dir = numpy.linspace(-numpy.pi, numpy.pi, n, endpoint = False)
    Input = numpy.zeros((len(pref_dir)), dtype = numpy.float64)
    Neuron_1 = numpy.zeros((n), dtype = numpy.float64)

    outs = -3 * numpy.ones((int(numpy.fix(numpy.true_divide((24 - 5), 2)))), dtype = numpy.float64)
    side = numpy.array([0.005, 0.005], dtype = numpy.float64)
    center = -1*numpy.array([0.0068], dtype = numpy.float64)
    synapses = numpy.hstack((outs, side, center, numpy.flip(side,0), outs))

    distances_vector = [0, 0]
    angles_vector = [0, 0]

    Neuron = numpy.zeros((n), dtype = numpy.float64)
    PSP = numpy.zeros((n), dtype = numpy.float64)
    Tau = 10


    z = 0
    j = 0


    while not rospy.is_shutdown():



        if t == 300:

            Neuron_1 = Input
            distance = DIST
            angle = ANGL

            distances_vector[1] = distance
            angles_vector[1] = angle


        if t == 900:

            distance = perceived_distance
            angle = perceived_dir
            distances_vector[0] = true_distance
            angles_vector[0] = true_dir

            Neuron_1 = numpy.zeros((n))
            Neuron = numpy.zeros((n))

            if pub_enable == 1:
                pub.publish(numpy.array([true_dir, true_distance, perceived_dir, perceived_distance]))
                print([true_dir, true_distance, perceived_dir, perceived_distance])
                pub_enable = 0

            z += 1



        Input = distance * numpy.cos((pref_dir - numpy.radians(angle)))


        conv = cir_conv(synapses, Neuron)
        PSP = Input + conv * (t > 600) + Neuron_1
        PSP = PSP * (PSP > 0)

        Neuron = Neuron + (2 / Tau) * (- Neuron + PSP)

        pos_max = numpy.argmax(Neuron)
        max_fir = Neuron[pos_max]
        plotorient = numpy.linspace(-180, 180, n, endpoint = False)
        Neu = numpy.hstack((Neuron, Neuron, Neuron))
        plto = numpy.hstack((plotorient, plotorient, plotorient))
        perceived_dir = parab_int(Neu[len(Neuron) + pos_max - 1], max_fir, Neu[len(Neuron) + pos_max + 1], plto[len(Neuron) + pos_max], 360/n)

        true_dir = numpy.degrees(numpy.arctan2(numpy.sum(distances_vector * numpy.sin(numpy.radians(angles_vector))), numpy.sum(distances_vector * numpy.cos(numpy.radians(angles_vector)))))
        perceived_distance = max_fir

        true_distance = numpy.sum(distances_vector * numpy.cos(numpy.radians(angles_vector) - numpy.radians(true_dir)))


        if true_distance<0:
            true_distance = true_distance * (-1)
            true_dir += 180

        if perceived_dir < 0:
            perceived_dir += 360

        if true_dir < 0:
            true_dir += 360

        if true_dir > 360:
            y = numpy.fix(true_dir / 360)
            true_dir -= y*360





        t += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        VectorSum()
    except rospy.ROSInterruptException:
        pass
