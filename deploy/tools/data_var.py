# Q字典
import numpy as np
D2R = np.pi/180
R2D = 180/np.pi

num_dofs = 12
enable_qt = True
enable_qc = True
enable_taut = False
enable_tauc = True
qt = np.ones(num_dofs, dtype = np.int32) * enable_qt
qc = np.ones(num_dofs, dtype = np.int32) * enable_qc
tt = np.ones(num_dofs, dtype = np.int32) * enable_taut
tc = np.ones(num_dofs, dtype = np.int32) * enable_tauc

Q = {#  窗口         线型    颜色      缩放      变量名
'time': [0,         '-',    'k',       1,     'time'],

# 'L_hip_pitch_qt'  : [1*qt[0],         '-.',   'k',     R2D,     'target_q[0]'],
# 'L_hip_pitch_qc'  : [1*qc[0],         '-',    'g',     R2D,     'q[0]'],
# 'L_hip_pitch_taut': [1*tt[0],         '-.',   'r',       1,     'tau[0]'],
'L_hip_pitch_tauc': [1*tc[0],         '-',    'b',       1,     'tauc[0]'],

# 'L_hip_roll_qt'  : [2*qt[1],         '-.',   'k',     R2D,     'target_q[1]'],
# 'L_hip_roll_qc'  : [2*qc[1],         '-',    'g',     R2D,     'q[1]'],
# 'L_hip_roll_taut': [2*tt[1],         '-.',   'r',       1,     'tau[1]'],
'L_hip_roll_tauc': [2*tc[1],         '-',    'b',       1,     'tauc[1]'],

# 'L_hip_yaw_qt'  : [3*qt[2],         '-.',   'k',     R2D,     'target_q[2]'],
# 'L_hip_yaw_qc'  : [3*qc[2],         '-',    'g',     R2D,     'q[2]'],
# 'L_hip_yaw_taut': [3*tt[2],         '-.',   'r',       1,     'tau[2]'],
'L_hip_yaw_tauc': [3*tc[2],         '-',    'b',       1,     'tauc[2]'],

# 'L_knee_qt'  : [4*qt[3],         '-.',   'k',     R2D,     'target_q[3]'],
# 'L_knee_qc'  : [4*qc[3],         '-',    'g',     R2D,     'q[3]'],
# 'L_knee_taut': [4*tt[3],         '-.',   'r',       1,     'tau[3]'],
'L_knee_tauc': [4*tc[3],         '-',    'b',       1,     'tauc[3]'],

# 'L_ankle_pitch_qt'  : [5*qt[4],         '-.',   'k',     R2D,     'target_q[4]'],
# 'L_ankle_pitch_qc'  : [5*qc[4],         '-',    'g',     R2D,     'q[4]'],
# 'L_ankle_pitch_taut': [5*tt[4],         '-.',   'r',       1,     'tau[4]'],
'L_ankle_pitch_tauc': [5*tc[4],         '-',    'b',       1,     'tauc[4]'],

# 'L_ankle_roll_qt'  : [6*qt[5],         '-.',   'k',     R2D,     'target_q[5]'],
# 'L_ankle_roll_qc'  : [6*qc[5],         '-',    'g',     R2D,     'q[5]'],
# 'L_ankle_roll_taut': [6*tt[5],         '-.',   'r',       1,     'tau[5]'],
'L_ankle_roll_tauc': [6*tc[5],         '-',    'b',       1,     'tauc[5]'],

# 'R_hip_pitch_qt'  : [7*qt[6],         '-.',   'k',     R2D,     'target_q[6]'],
# 'R_hip_pitch_qc'  : [7*qc[6],         '-',    'g',     R2D,     'q[6]'],
# 'R_hip_pitch_taut': [7*tt[6],         '-.',   'r',       1,     'tau[6]'],
'R_hip_pitch_tauc': [7*tc[6],         '-',    'b',       1,     'tauc[6]'],

# 'R_hip_roll_qt'  : [8*qt[7],         '-.',   'k',     R2D,     'target_q[7]'],
# 'R_hip_roll_qc'  : [8*qc[7],         '-',    'g',     R2D,     'q[7]'],
# 'R_hip_roll_taut': [8*tt[7],         '-.',   'r',       1,     'tau[7]'],
'R_hip_roll_tauc': [8*tc[7],         '-',    'b',       1,     'tauc[7]'],

# 'R_hip_yaw_qt'  : [9*qt[8],         '-.',   'k',     R2D,     'target_q[8]'],
# 'R_hip_yaw_qc'  : [9*qc[8],         '-',    'g',     R2D,     'q[8]'],
# 'R_hip_yaw_taut': [9*tt[8],         '-.',   'r',       1,     'tau[8]'],
'R_hip_yaw_tauc': [9*tc[8],         '-',    'b',       1,     'tauc[8]'],

# 'R_knee_qt'  : [10*qt[9],         '-.',   'k',     R2D,     'target_q[9]'],
# 'R_knee_qc'  : [10*qc[9],         '-',    'g',     R2D,     'q[9]'],
# 'R_knee_taut': [10*tt[9],         '-.',   'r',       1,     'tau[9]'],
'R_knee_tauc': [10*tc[9],         '-',    'b',       1,     'tauc[9]'],

# 'R_ankle_pitch_qt'  : [11*qt[10],         '-.',   'k',     R2D,     'target_q[10]'],
# 'R_ankle_pitch_qc'  : [11*qc[10],         '-',    'g',     R2D,     'q[10]'],
# 'R_ankle_pitch_taut': [11*tt[10],         '-.',   'r',       1,     'tau[10]'],
'R_ankle_pitch_tauc': [11*tc[10],         '-',    'b',       1,     'tauc[10]'],

# 'R_ankle_roll_qt'  : [12*qt[11],         '-.',   'k',     R2D,     'target_q[11]'],
# 'R_ankle_roll_qc'  : [12*qc[11],         '-',    'g',     R2D,     'q[11]'],
# 'R_ankle_roll_taut': [12*tt[11],         '-.',   'r',       1,     'tau[11]'],
'R_ankle_roll_tauc': [12*tc[11],         '-',    'b',       1,     'tauc[11]'],

'L_hip_pitch_dq': [13,         '-',   'k',     1,     'dq[0]'],
'L_hip_roll':     [13,         '-',   'g',     1,     'dq[1]'],
'L_hip_yaw' :     [13,         '-',   'r',     1,     'dq[2]'],
'L_knee'    :     [13,         '-',   'b',     1,     'dq[3]'],
'L_ankle_pitch':  [13,         '-',   'm',     1,     'dq[4]'],
'L_ankle_roll':   [13,         '-',   'y',     1,     'dq[5]'],

'R_hip_pitch_dq': [14,         '-',   'k',     1,     'dq[6]'],
'R_hip_roll':     [14,         '-',   'g',     1,     'dq[7]'],
'R_hip_yaw' :     [14,         '-',   'r',     1,     'dq[8]'],
'R_knee'    :     [14,         '-',   'b',     1,     'dq[9]'],
'R_ankle_pitch':  [14,         '-',   'm',     1,     'dq[10]'],
'R_ankle_roll':   [14,         '-',   'y',     1,     'dq[11]'],

# 'eulerR': [15,       '-.',   'k',     R2D,     'eu_ang[0]'],
# 'eulerP': [15,       '-',    'g',     R2D,     'eu_ang[1]'],
'gyroR' : [15,       '-.',   'r',     R2D,    'gyro[0]'],
'gyroP' : [15,       '-',    'g',     R2D,    'gyro[1]'],

# 'vx_command': [16,     '-.',   'r',     1,    'vx_command'],
# 'vx_current': [16,     '-',    'g',     1,    'vx_current'],

# 'hip_roll_dq':     [17,         '-',   'k',     1,     'dq[1]'],
# 'hip_yaw_dq' :     [17,         '-',   'g',     1,     'dq[2]'],
# 'hip_ry_dq_plus':  [17,         '-',   'b',     1,     'abs(dq[1]) + abs(dq[2])'],
#
# 'knee_dq':           [18,         '-',   'k',     1,     'dq[3]'],
# 'ankle_pitch_dq' :   [18,         '-',   'g',     1,     'dq[4]'],
# 'knee_ankle_dq_plus':[18,         '-',   'b',     1,     'abs(dq[3]) + abs(dq[4])'],
#
# 'hip_roll_tau':     [19,         '-',   'k',     1,     'tauc[1]'],
# 'hip_yaw_tau' :     [19,         '-',   'g',     1,     'tauc[2]'],
# 'hip_ry_tau_plus':  [19,         '-',   'b',     1,     'abs(tauc[1]) + abs(tauc[2])'],
#
# 'knee_tau':           [20,         '-',   'k',     1,     'tauc[3]'],
# 'ankle_pitch_tau' :   [20,         '-',   'g',     1,     'tauc[4]'],
# 'knee_ankle_tau_plus':[20,         '-',   'b',     1,     'abs(tauc[3]) + abs(tauc[4])'],

# 'hip_roll_dq_m':     [17,         '-',   'k',     1,     'dq_m[1]'],
# 'hip_yaw_dq_m' :     [17,         '-',   'g',     1,     'dq_m[2]'],
#
# 'knee_dq_m':           [18,         '-',   'k',     1,     'dq_m[3]'],
# 'ankle_pitch_dq_m' :   [18,         '-',   'g',     1,     'dq_m[4]'],
#
# 'hip_roll_tau_m':     [19,         '-',   'k',     1,     'tau_m[1]'],
# 'hip_yaw_tau_m' :     [19,         '-',   'g',     1,     'tau_m[2]'],
#
# 'knee_tau_m':           [20,         '-',   'k',     1,     'tau_m[3]'],
# 'ankle_pitch_tau_m' :   [20,         '-',   'g',     1,     'tau_m[4]'],
}
QKey = Q.keys()
QKey_list = list(QKey)
var_list = []
# Q字典生成data字典
data_dict = {}
for key in QKey_list:
    data_dict[key] =[]
    var_list.append(Q[key][3]) #保存变量名
