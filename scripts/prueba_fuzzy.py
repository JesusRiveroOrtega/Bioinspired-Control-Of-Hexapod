import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl
# Generate universe variables
#   * Quality and service on subjective ranges [0, 10]
#   * Tip has a range of [0, 25] in units of percentage points
error = ctrl.Antecedent(np.arange(-2, 0.001, 0.001), 'error')
d_error = ctrl.Antecedent(np.arange(-2000, 2001, 1), 'd_error')
rotation_velocity_increase = ctrl.Consequent(np.arange(-0.1, 0.101, 0.001), 'rotation_velocity_increase')



error['VH'] = fuzz.trimf(error.universe, [-2.0, -2.0, -1.5])
error['H'] = fuzz.trimf(error.universe, [-2.0, -1.5, -1.0])
error['M'] = fuzz.trimf(error.universe, [-1.5, -1.0, -0.5])
error['L'] = fuzz.trimf(error.universe, [-1.0, -0.5,  0.0])
error['VL'] = fuzz.trimf(error.universe, [-0.5,  0.0,  0.0])

d_error['VN'] = fuzz.trimf(d_error.universe, [-2000, -2000, -1000])
d_error['N'] = fuzz.trimf(d_error.universe, [-2000, -1000,    -0])
d_error['M'] = fuzz.trimf(d_error.universe, [-1000,     0,  1000])
d_error['P'] = fuzz.trimf(d_error.universe, [    0,  1000,  2000])
d_error['VP'] = fuzz.trimf(d_error.universe, [ 1000,  2000,  2000])

rotation_velocity_increase['N'] = fuzz.trimf(rotation_velocity_increase.universe, [-0.10, -0.10, 0.00])
rotation_velocity_increase['Z'] = fuzz.trimf(rotation_velocity_increase.universe, [-0.10, -0.00, 0.10])
rotation_velocity_increase['P'] = fuzz.trimf(rotation_velocity_increase.universe, [-0.00,  0.10, 0.10])


rule_11 = ctrl.Rule(error['VH'] & d_error['VN'], rotation_velocity_increase['N'])
rule_12 = ctrl.Rule(error['VH'] & d_error['N'], rotation_velocity_increase['N'])
rule_13 = ctrl.Rule(error['VH'] & d_error['M'], rotation_velocity_increase['Z'])
rule_14 = ctrl.Rule(error['VH'] & d_error['P'], rotation_velocity_increase['P'])
rule_15 = ctrl.Rule(error['VH'] & d_error['VP'], rotation_velocity_increase['P'])

rule_21 = ctrl.Rule(error['H'] & d_error['VN'], rotation_velocity_increase['N'])
rule_22 = ctrl.Rule(error['H'] & d_error['N'], rotation_velocity_increase['N'])
rule_23 = ctrl.Rule(error['H'] & d_error['M'], rotation_velocity_increase['Z'])
rule_24 = ctrl.Rule(error['H'] & d_error['P'], rotation_velocity_increase['P'])
rule_25 = ctrl.Rule(error['H'] & d_error['VP'], rotation_velocity_increase['P'])

rule_31 = ctrl.Rule(error['M'] & d_error['VN'], rotation_velocity_increase['N'])
rule_32 = ctrl.Rule(error['M'] & d_error['N'], rotation_velocity_increase['N'])
rule_33 = ctrl.Rule(error['M'] & d_error['M'], rotation_velocity_increase['Z'])
rule_34 = ctrl.Rule(error['M'] & d_error['P'], rotation_velocity_increase['P'])
rule_35 = ctrl.Rule(error['M'] & d_error['VP'], rotation_velocity_increase['P'])

rule_41 = ctrl.Rule(error['L'] & d_error['VN'], rotation_velocity_increase['N'])
rule_42 = ctrl.Rule(error['L'] & d_error['N'], rotation_velocity_increase['N'])
rule_43 = ctrl.Rule(error['L'] & d_error['M'], rotation_velocity_increase['Z'])
rule_44 = ctrl.Rule(error['L'] & d_error['P'], rotation_velocity_increase['P'])
rule_45 = ctrl.Rule(error['L'] & d_error['VP'], rotation_velocity_increase['P'])

rule_51 = ctrl.Rule(error['VL'] & d_error['VN'], rotation_velocity_increase['N'])
rule_52 = ctrl.Rule(error['VL'] & d_error['N'], rotation_velocity_increase['N'])
rule_53 = ctrl.Rule(error['VL'] & d_error['M'], rotation_velocity_increase['Z'])
rule_54 = ctrl.Rule(error['VL'] & d_error['P'], rotation_velocity_increase['P'])
rule_55 = ctrl.Rule(error['VL'] & d_error['VP'], rotation_velocity_increase['P'])


rotation_velocity_increase_ctrl = ctrl.ControlSystem([rule_11, rule_12, rule_13, rule_14, rule_15,
                                   rule_21, rule_22, rule_23, rule_24, rule_25,
                                   rule_31, rule_32, rule_33, rule_34, rule_35,
                                   rule_41, rule_42, rule_43, rule_44, rule_45,
                                   rule_51, rule_52, rule_53, rule_54, rule_55])
rotation_velocity_increase_sim = ctrl.ControlSystemSimulation(rotation_velocity_increase_ctrl)
rotation_velocity_increase_sim.input['error'] = -0.5
rotation_velocity_increase_sim.input['d_error'] = -2/0.01
rotation_velocity_increase_sim.compute()
print rotation_velocity_increase_sim.output['rotation_velocity_increase']
rotation_velocity_increase.view(sim=rotation_velocity_increase_sim)
plt.show()
