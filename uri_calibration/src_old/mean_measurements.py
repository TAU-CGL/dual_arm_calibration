import numpy as np

def find_outliers_iqr(data, threshold=1.5):
    q1 = np.percentile(data, 25)
    q3 = np.percentile(data, 75)
    iqr = q3 - q1
    
    # Standard threshold is 1.5 * IQR
    lower_bound = q1 - (threshold * iqr)
    upper_bound = q3 + (threshold * iqr)
    
    outliers = data[(data < lower_bound) | (data > upper_bound)]
    indices = np.where((data < lower_bound) | (data > upper_bound))[0]
    
    return outliers, indices

x_plus = np.zeros((4, 6))
y_minus = np.zeros((4, 6))
y_plus = np.zeros((4, 6))
x_minus = np.zeros((4, 6))
z_plus = np.zeros((4, 6))
z_minus = np.zeros((4, 6))

# ----------------1ST!!!-----------------
GT = [0.601803, -1.002126, 0.27286, 0.004082, -0.00624, -0.485771]
x_plus[0] = [0.569808, -0.997523, 0.219629, 0.029892, 0.084198, -0.506875]
x_plus[1] = [0.561966, -1.013869, 0.263029, 0.038923, -0.032901, -0.556027]
x_plus[2] = [0.660636, -0.971589, 0.269265, 0.025457, -0.026963, -0.402441]
x_plus[3] = [0.564679, -1.001628, 0.224626, 0.034047, 0.067433, -0.521979]

y_minus[0] = [0.54236, -1.012559, 0.284901, -0.010474, -0.035448, -0.593342]
y_minus[1] = [0.665674, -0.981633, 0.255127, 0.04774, -0.027328, -0.365185]
y_minus[2] = [0.591308, -0.992602, 0.189959, 0.145253, 0.052208, -0.499523]
y_minus[3] = [0.606132, -1.002549, 0.323657, -0.054894, -0.09712, -0.47947]

x_minus[0] = [0.522783, 0.07919, 0.473834, -2.828483, -0.994389, 0.341332]
x_minus[1] = [0.63287, -0.975181, 0.256503, 0.024426, 0.014908, -0.411482]
x_minus[2] = [0.562899, -1.021785, 0.266004, 0.001748, 0.012643, -0.560459]
x_minus[3] = [0.609664, -1.007207, 0.319758, -0.014555, -0.10697, -0.488872]

z_plus[0] = [0.612164, -0.977462, 0.215298, 0.074271, 0.071055, -0.443728]
z_plus[1] = [0.62534, -0.993767, 0.329607, -0.040962, -0.099591, -0.446454]
z_plus[2] = [0.613624, -0.985743, 0.246433, 0.100128, -0.024029, -0.467501]
z_plus[3] = [0.614627, -1.002489, 0.336617, -0.044069, -0.112885, -0.470755]

z_minus[0] = [0.614025, -0.990571, 0.249015, 0.003187, 0.040997, -0.45714]
z_minus[1] = [0.623711, -1.000919, 0.323104, -0.059008, -0.034941, -0.456141]
z_minus[2] = [0.616186, -0.978227, 0.212484, 0.058287, 0.045793, -0.454802]
z_minus[3] = [0.597639, -0.994688, 0.235971, -0.018408, 0.09129, -0.464734]
# z_minus[4] = [0.635561, -0.980367, 0.253348, 0.094197, -0.07602, -0.460512]

# ----------------2ND!!!-----------------
# GT = [0.620444, -0.55571, 0.271564, 0.012676, -0.004661, 0.592253]
# x_plus[0] = [0.581857, -0.594192, 0.24898, 0.035675, 0.038134, 0.474671]
# x_plus[1] = [0.61924, -0.513179, 0.259006, 0.032793, 0.012551, 0.665495]
# x_plus[2] = [0.568156, -0.536359, 0.221094, 0.081003, 0.087014, 0.581208]
# x_plus[3] = [0.653045, -0.531867, 0.292297, 0.00842, -0.064125, 0.619542]

# y_minus[0] = [0.620713, -0.590225, 0.318501, -0.024649, -0.118606, 0.508179]
# y_minus[1] = [0.637714, -0.52718, 0.290791, 0.002549, -0.066488, 0.661687]
# y_minus[2] = [0.627763, -0.561945, 0.296868, -0.101583, 8.4e-05, 0.614891]
# y_minus[3] = [0.62754, -0.54317, 0.276555, 0.07045, -0.071178, 0.591794]

# y_plus[0] = [0.653355, -0.532821, 0.291461, -0.029656, -0.036865, 0.671856]
# y_plus[1] = [0.581669, -0.600583, 0.26612, -0.010271, 0.035226, 0.487308]
# y_plus[2] = [0.634361, -0.480245, 0.251339, 0.120205, -0.044395, 0.655582]
# y_plus[3] = [0.619901, -0.581759, 0.287717, -0.093569, 0.053482, 0.629382]

# z_plus[0] = [0.651025, -0.538783, 0.286417, 0.032791, -0.10186, 0.571918]
# z_plus[1] = [0.615791, -0.615235, 0.31683, -0.116769, 0.022693, 0.568724]
# z_plus[2] = [0.577219, -0.525653, 0.21809, 0.097433, 0.0588, 0.582558]
# z_plus[3] = [0.572793, -0.563048, 0.240794, 0.012468, 0.108382, 0.601223]

# z_minus[0] = [0.585857, -0.564162, 0.209906, -0.009764, 0.168561, 0.607444]
# z_minus[1] = [0.64455, -0.542819, 0.311474, -0.017593, -0.079956, 0.620011]
# z_minus[2] = [0.612804, -0.573931, 0.284238, -0.077242, 0.061825, 0.599609]
# z_minus[3] = [0.60248, -0.529262, 0.199514, 0.107462, 0.05522, 0.606331]


total_vec = np.zeros((6, 20))
sorted_total_vec = np.zeros((6, 20))
for i in range(6):
    total_vec[i][0:4] = x_plus[:,i]
    total_vec[i][4:8] = y_minus[:,i]
    total_vec[i][8:12] = x_minus[:,i]
    # total_vec[i][8:12] = y_plus[:,i]
    total_vec[i][12:16] = z_plus[:,i]
    total_vec[i][16:20] = z_minus[:,i]
    sorted_total_vec[i] = np.sort(total_vec[i])

# Analyze X values
outliers, indices = find_outliers_iqr(sorted_total_vec[0], threshold=0.5)
clean_vec = np.delete(sorted_total_vec[0], indices)

max_x_value = clean_vec[-1]
min_x_value = clean_vec[0]
mean_x_value = np.mean(clean_vec)
x_margin = max(abs(max_x_value - GT[0]), abs(min_x_value - GT[0]))
mean_error_x = abs(mean_x_value - GT[0])
print("--------------------------------------------")
print("Outliers in X values:", outliers)
print("Indices of outliers in X values:", indices)
print("Max X Value:", max_x_value)
print("Min X Value:", min_x_value)
print("Mean X Value:", mean_x_value)
print("Ground Truth X Value:", GT[0])
print("X Margin:", x_margin)
print("Mean Error in X:", mean_error_x)

# Analyze Y values
outliers, indices = find_outliers_iqr(sorted_total_vec[1])
clean_vec = np.delete(sorted_total_vec[1], indices)

max_y_value = clean_vec[-1]
min_y_value = clean_vec[0]
mean_y_value = np.mean(clean_vec)
y_margin = max(abs(max_y_value - GT[1]), abs(min_y_value - GT[1]))
mean_error_y = abs(mean_y_value - GT[1])
print("--------------------------------------------")
print("Outliers in Y values:", outliers)
print("Indices of outliers in Y values:", indices)
print("Max Y Value:", max_y_value)
print("Min Y Value:", min_y_value)
print("Mean Y Value:", mean_y_value)
print("Ground Truth Y Value:", GT[1])
print("Y Margin:", y_margin)
print("Mean Error in Y:", mean_error_y)

# Analyze Z values
outliers, indices = find_outliers_iqr(sorted_total_vec[2], threshold=3.0)
clean_vec = np.delete(sorted_total_vec[2], indices)

max_z_value = clean_vec[-1]
min_z_value = clean_vec[0]
mean_z_value = np.mean(clean_vec)
z_margin = max(abs(max_z_value - GT[2]), abs(min_z_value - GT[2]))
mean_error_z = abs(mean_z_value - GT[2])
print("--------------------------------------------")
print("Outliers in Z values:", outliers)
print("Indices of outliers in Z values:", indices)
print("Max Z Value:", max_z_value)
print("Min Z Value:", min_z_value)
print("Mean Z Value:", mean_z_value)
print("Ground Truth Z Value:", GT[2])
print("Z Margin:", z_margin)
print("Mean Error in Z:", mean_error_z)

# Analyze R_x values
outliers, indices = find_outliers_iqr(sorted_total_vec[3], threshold=1.0)
clean_vec = np.delete(sorted_total_vec[3], indices)
max_x_value = clean_vec[-1]
min_x_value = clean_vec[0]
mean_x_value = np.mean(clean_vec)
x_margin = max(abs(max_x_value - GT[3]), abs(min_x_value - GT[3]))
mean_error_x = abs(mean_x_value - GT[3])
print("--------------------------------------------")
print("Outliers in R_x values:", outliers)
print("Indices of outliers in R_x values:", indices)
print("Max R_x Value:", max_x_value)
print("Min R_x Value:", min_x_value)
print("Mean R_x Value:", mean_x_value)
print("Ground Truth R_x Value:", GT[3])
print("R_x Margin:", x_margin)
print("Mean Error in R_x:", mean_error_x)

# Analyze R_y values
outliers, indices = find_outliers_iqr(sorted_total_vec[4], threshold=1.0)
clean_vec = np.delete(sorted_total_vec[4], indices)
max_y_value = clean_vec[-1]
min_y_value = clean_vec[0]
mean_y_value = np.mean(clean_vec)
y_margin = max(abs(max_y_value - GT[4]), abs(min_y_value - GT[4]))
mean_error_y = abs(mean_y_value - GT[4])
print("--------------------------------------------")
print("Outliers in R_y values:", outliers)
print("Indices of outliers in R_y values:", indices)
print("Max R_y Value:", max_y_value)
print("Min R_y Value:", min_y_value)
print("Mean R_y Value:", mean_y_value)
print("Ground Truth R_y Value:", GT[4])
print("R_y Margin:", y_margin)
print("Mean Error in R_y:", mean_error_y)

# Analyze R_z values
outliers, indices = find_outliers_iqr(sorted_total_vec[5], threshold=1.0)
clean_vec = np.delete(sorted_total_vec[5], indices)
max_z_value = clean_vec[-1]
min_z_value = clean_vec[0]
mean_z_value = np.mean(clean_vec)
z_margin = max(abs(max_z_value - GT[5]), abs(min_z_value - GT[5]))
mean_error_z = abs(mean_z_value - GT[5])
print("--------------------------------------------")
print("Outliers in R_z values:", outliers)
print("Indices of outliers in R_z values:", indices)
print("Max R_z Value:", max_z_value)
print("Min R_z Value:", min_z_value)
print("Mean R_z Value:", mean_z_value)
print("Ground Truth R_z Value:", GT[5])
print("R_z Margin:", z_margin)
print("Mean Error in R_z:", mean_error_z)